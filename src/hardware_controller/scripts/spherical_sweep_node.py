#!/usr/bin/env python3
"""
spherical_sweep_node - sweep sferyczny obiema osiami głowicy, na realnym sprzęcie.

DWA TRYBY PRACY (parametr `mode`):

  "step"       - "dojedź, ustój, zbieraj": siatka azymut x elewacja, w każdym
                 węźle postój. Tak powstały wszystkie chmury do 2026-09-05.
  "continuous" - wiersz elewacji przejeżdżany BEZ ZATRZYMANIA, azymut
                 przestawiany między wierszami. Domyślny od 2026-09-16.

DLACZEGO TRYB CIĄGŁY W OGÓLE WOLNO WŁĄCZYĆ. Pierwotny sweep stał w każdym
punkcie nie z ostrożności, tylko dlatego, że Sharp uśredniał okno ~50 ms, a
dalmierz JRT LDB1 mielił pojedynczy strzał 0,5 s - w ruchu punkt rozmazywał się
po łuku. Oba założenia upadły w pomiarach:

  * moduł NIE CAŁKUJE po całym oknie (test amplitudy profilu przy 12/24/48/72
    st/s, 2026-09-05): przy łuku 35 st amplituda spadła o 5%, a nie do 1/3.
    Te setki milisekund to narzut i obróbka, światło zbierane jest krótko;
  * test PAROWANY (2026-09-05) dał przy 18,6 st/s tę samą odległość co
    dojedź-i-ustój: bias -0,9 mm, sigma 16,8 mm - a te 16,8 mm to w większości
    LUZ mechaniczny (<= 0,5 st), nie ruch.

Zysk jest za to duży: znika settle + dwell + dojazd, czyli ~2 s na punkt.
Z dalmierzem M703A (8,15 Hz) półsfera co 3 st schodzi z godzin do ~8 minut.

CENĄ JEST CZAS. W bezruchu stempel pomiaru mógł być byle jaki, bo kąt się nie
zmieniał. W ruchu błąd czasu JEST błędem kąta: przy 24 st/s 10 ms to 0,24 st.
Dlatego tryb ciągły ma sens wyłącznie z czujnikiem stemplowanym na Nano
(M703aLaserSensor) i z kotwicą zmierzoną testem rewersyjnym.

KAŻDY RUCH LICZY FIRMWARE SERWA (od 2026-09-17). Węzeł ustawia prędkość
i przyspieszenie profilu przez /profile_velocity_controller i
/profile_acceleration_controller, po czym wysyła SAM PUNKT KOŃCOWY - trapez
prędkości liczy serwo z częstotliwością 1 kHz.

Wcześniej węzeł strumieniował pozycję co 20 ms. Przy włączonym profilu serwo
planowało wtedy osobny mini-trapez przy KAŻDYM kroku, a głowica "klatkowała"
(obserwacja usera). Dla rekonstrukcji nic się nie zmienia: kąt pod pomiar i tak
pochodzi z enkoderów, nie z zadanej trajektorii.

Nadal obowiązuje zasada z 2026-09-05: głowica stoi luźno na małej podstawie,
więc żaden ruch nie może iść z pełną prędkością serwa - zawsze z jawnym,
umiarkowanym przyspieszeniem profilu. Rampa firmware'u jest TRAPEZOWA
(przyspieszenie przeskakuje z 0 na zadane), a nie esowata jak w wersji
strumieniowanej; gdyby start wiersza szarpał, pierwszym ruchem jest obniżenie
accel_deg_s2.

Publikuje:
  /forward_position_controller/commands  (Float64MultiArray) - pozycje jointów [rad]
  /sweep/collecting                      (Bool)   - true, gdy głowica stoi i wolno zbierać
  /sweep/state                           (String) - "running" / "done"

Subskrybuje:
  /joint_states                          (JointState) - do wykrycia dojazdu

Parametry:
  azimuth_joint    (str, xm540_joint_z) - nazwa jointa azymutu (człon 1, serwo ID 2)
  elevation_joint  (str, xm540_joint)   - nazwa jointa elewacji (głowica, serwo ID 1)
  joint_order      (str[], [xm540_joint, xm540_joint_z]) - kolejność w komendzie;
                   MUSI odpowiadać liście `joints` forward_position_controller,
                   bo kontroler dostaje goły wektor bez nazw i nie ma jak wykryć
                   zamiany osi - objawem byłby sweep obrócony o 90 stopni.
  az_min_deg/az_max_deg/az_step_deg      - siatka azymutu
  el_min_deg/el_max_deg/el_step_deg      - siatka elewacji
  settle_time      (float, 0.4) - ile czekać po dojeździe, zanim zaczniemy zbierać [s]
  dwell_time       (float, 0.4) - ile DODATKOWO zbierać po pierwszym świeżym pomiarze [s]
  wait_for_measurement (bool, True) - czekać na fakt zamiast na zegar (patrz niżej)
  max_wait         (float, 5.0) - górna granica czekania na pomiar w punkcie [s]

CZEKANIE NA POMIAR, NIE NA ZEGAR
Dalmierz JRT mierzy wg instrukcji 0,1-4 s - czas zależy od celu, nie jest stały.
Odmierzanie stałego dwell_time byłoby więc zgadywaniem w obie strony: na łatwym
celu marnuje czas, na trudnym gubi punkt. Zamiast tego czekamy, aż wróci pomiar
ROZPOCZĘTY PO dojeździe, i dopiero wtedy ruszamy głowicę.

Rozstrzyga o tym pole z /laser/raw, w którym wtyczka podaje moment WYSŁANIA
żądania. Sam moment powrotu odpowiedzi nie wystarcza: strzał mógł się zacząć
jeszcze przy ruchomej głowicy i wrócić już po zatrzymaniu - trafiłby wtedy do
chmury pod współrzędnymi pozycji docelowej i rozmazał ją wzdłuż toru ruchu.
  tolerance_deg    (float, 0.5) - próg uznania pozycji za osiągniętą
  arrival_timeout  (float, 5.0) - po tylu sekundach jedziemy dalej mimo braku dojazdu [s]
  serpentine       (bool, True) - co drugi wiersz w odwrotną stronę (krótsza droga)
  return_to_zero   (bool, True) - czy wrócić do zera po sweepie i przy zamknięciu

Parametry trybu ciągłego:
  mode             (str, continuous) - "continuous" albo "step"
  sweep_speed_deg_s (float, 24.0) - prędkość przejazdu wiersza elewacji [st/s].
                   Razem z tempem czujnika wyznacza GĘSTOŚĆ punktów wzdłuż
                   wiersza: odstęp = prędkość / tempo. 24 st/s przy 8,15 Hz daje
                   2,9 st, czyli tyle, ile dawała siatka co 3 st - tylko szybciej.
  travel_speed_deg_s (float, 40.0) - prędkość przejazdów NIEZBIERAJĄCYCH
                   (dojazd na start wiersza, zmiana azymutu, parkowanie).
  accel_deg_s2     (float, 60.0) - przyspieszenie profilu serwa [st/s^2]; rozbieg
                   do 24 st/s trwa 0,4 s. Kwant rejestru: 21,5 st/s^2.
  Prędkość profilu ma kwant 1,374 st/s - 24 st/s jedzie naprawdę jako 23,4.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float64MultiArray, String


def _grid(lo, hi, step):
    """Siatka od lo do hi włącznie, z krokiem step (odporna na hi < lo i step <= 0)."""
    if step <= 0.0:
        raise ValueError("krok siatki musi być dodatni")
    if hi < lo:
        lo, hi = hi, lo
    # floor, nie round: przy kroku niedzielącym zakresu (np. -15..15 co 4)
    # zaokrąglenie w górę wypchnęłoby ostatni punkt POZA zadany zakres, czyli
    # głowica pojechałaby dalej, niż kazano. Zamiast tego domykamy prawy koniec
    # jawnie, żeby skan sięgał dokładnie do hi.
    n = int(math.floor((hi - lo) / step + 1e-9))
    pts = [lo + i * step for i in range(n + 1)]
    if hi - pts[-1] > 1e-9:
        pts.append(hi)
    return pts


class SphericalSweepNode(Node):
    def __init__(self):
        super().__init__("spherical_sweep_node")

        self.declare_parameter("azimuth_joint", "xm540_joint_z")
        self.declare_parameter("elevation_joint", "xm540_joint")
        self.declare_parameter("joint_order", ["xm540_joint", "xm540_joint_z"])

        self.declare_parameter("az_min_deg", -15.0)
        self.declare_parameter("az_max_deg", 15.0)
        self.declare_parameter("az_step_deg", 2.0)
        self.declare_parameter("el_min_deg", -15.0)
        self.declare_parameter("el_max_deg", 15.0)
        self.declare_parameter("el_step_deg", 2.0)

        self.declare_parameter("settle_time", 0.4)
        self.declare_parameter("dwell_time", 0.4)
        self.declare_parameter("wait_for_measurement", True)
        self.declare_parameter("max_wait", 5.0)
        self.declare_parameter("tolerance_deg", 0.5)
        self.declare_parameter("arrival_timeout", 5.0)
        self.declare_parameter("serpentine", True)
        self.declare_parameter("return_to_zero", True)

        self.declare_parameter("mode", "continuous")
        self.declare_parameter("sweep_speed_deg_s", 24.0)
        self.declare_parameter("travel_speed_deg_s", 40.0)
        self.declare_parameter("accel_deg_s2", 60.0)

        self._az_joint = self.get_parameter("azimuth_joint").value
        self._el_joint = self.get_parameter("elevation_joint").value
        self._order = list(self.get_parameter("joint_order").value)

        for name in (self._az_joint, self._el_joint):
            if name not in self._order:
                raise RuntimeError(
                    f"joint '{name}' nie występuje w joint_order={self._order} - "
                    f"komenda trafiłaby w nieistniejącą pozycję wektora")

        self._settle = self.get_parameter("settle_time").value
        self._dwell = self.get_parameter("dwell_time").value
        self._wait_meas = self.get_parameter("wait_for_measurement").value
        self._max_wait = self.get_parameter("max_wait").value
        # Moment rozpoczęcia ostatniego strzału, prosto z /laser/raw (pole z).
        self._last_shot_start = 0.0
        self._n_wait_timeout = 0
        self._tol = math.radians(self.get_parameter("tolerance_deg").value)
        self._timeout = self.get_parameter("arrival_timeout").value
        self._return_to_zero = self.get_parameter("return_to_zero").value

        self._mode = self.get_parameter("mode").value
        if self._mode not in ("continuous", "step"):
            raise RuntimeError(f"mode='{self._mode}' - dozwolone: continuous, step")
        self._sweep_speed = self.get_parameter("sweep_speed_deg_s").value
        self._travel_speed = self.get_parameter("travel_speed_deg_s").value
        self._accel = self.get_parameter("accel_deg_s2").value
        if min(self._sweep_speed, self._travel_speed, self._accel) <= 0.0:
            raise RuntimeError("prędkości i przyspieszenie muszą być dodatnie")

        az = _grid(self.get_parameter("az_min_deg").value,
                   self.get_parameter("az_max_deg").value,
                   self.get_parameter("az_step_deg").value)
        el = _grid(self.get_parameter("el_min_deg").value,
                   self.get_parameter("el_max_deg").value,
                   self.get_parameter("el_step_deg").value)

        serpentine = self.get_parameter("serpentine").value
        self._points = []
        for i, a in enumerate(az):
            row = el if (not serpentine or i % 2 == 0) else list(reversed(el))
            for e in row:
                self._points.append((a, e))

        # W trybie ciągłym siatka elewacji nie istnieje - wiersz to PRZEJAZD od
        # krańca do krańca, a gęstość punktów wzdłuż niego ustala tempo czujnika
        # razem z prędkością osi. Siatka azymutu zostaje, bo azymut nadal
        # przestawiamy skokowo.
        self._rows = []
        for i, a in enumerate(az):
            lo, hi = el[0], el[-1]
            self._rows.append((a, hi, lo) if (serpentine and i % 2) else (a, lo, hi))

        self._positions = {}
        self._stop = threading.Event()

        self._pub_pvel = self.create_publisher(
            Float64MultiArray, "/profile_velocity_controller/commands", 10)
        self._pub_pacc = self.create_publisher(
            Float64MultiArray, "/profile_acceleration_controller/commands", 10)
        self._pub_cmd = self.create_publisher(
            Float64MultiArray, "/forward_position_controller/commands", 10)
        self._pub_collecting = self.create_publisher(Bool, "/sweep/collecting", 10)
        # Stan sweepu z QoS transient_local: kolektor, który wstanie później,
        # i tak dostanie ostatnią wartość - inaczej mógłby przegapić "done"
        # i nigdy nie zapisać chmury.
        self._pub_state = self.create_publisher(
            String, "/sweep/state", QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE))

        self.create_subscription(JointState, "/joint_states", self._cb_joints, 10)
        self.create_subscription(Vector3Stamped, "/laser/raw", self._cb_raw, 10)

        if self._mode == "continuous":
            span = abs(el[-1] - el[0])
            row_time = self._profile_time(span, self._sweep_speed, self._accel)
            self.get_logger().info(
                f"Sweep CIĄGŁY: azymut {az[0]:.1f}..{az[-1]:.1f} st ({len(az)} wierszy), "
                f"elewacja {el[0]:.1f}..{el[-1]:.1f} st przejazdem po {row_time:.1f} s "
                f"przy {self._sweep_speed:.1f} st/s")
            self.get_logger().info(
                f"Gęstość wzdłuż wiersza zależy od tempa czujnika: przy 8,15 Hz odstęp "
                f"{self._sweep_speed / 8.15:.2f} st, czyli ~{span / (self._sweep_speed / 8.15):.0f} "
                f"punktów na wiersz. Sam przejazd: {len(az) * row_time / 60.0:.1f} min "
                f"(bez zmian azymutu)")
            threading.Thread(target=self._run, daemon=True).start()
            return

        total = len(self._points)
        per_point = self._settle + self._dwell
        self.get_logger().info(
            f"Sweep sferyczny: azymut {az[0]:.1f}..{az[-1]:.1f} st ({len(az)} poz.), "
            f"elewacja {el[0]:.1f}..{el[-1]:.1f} st ({len(el)} poz.), "
            f"razem {total} punktów")
        self.get_logger().info(
            f"Szacowany czas: co najmniej {total * per_point / 60.0:.1f} min "
            f"(bez dojazdu i bez czekania na pomiar; {per_point:.2f} s na punkt)"
            if not self._wait_meas else
            f"Czas zależy od czujnika: {per_point:.2f} s na punkt to tylko settle+dwell, "
            f"do tego dojazd i czekanie na pomiar (max {self._max_wait:.1f} s/punkt)")

        threading.Thread(target=self._run, daemon=True).start()

    # ------------------------------------------------------------------ joints

    def _cb_joints(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._positions[name] = pos

    def _at_target(self, az_rad, el_rad) -> bool:
        a = self._positions.get(self._az_joint)
        e = self._positions.get(self._el_joint)
        if a is None or e is None:
            return False
        return abs(a - az_rad) <= self._tol and abs(e - el_rad) <= self._tol

    # ------------------------------------------------------------------ ruch

    def _send(self, az_rad, el_rad):
        by_name = {self._az_joint: az_rad, self._el_joint: el_rad}
        msg = Float64MultiArray()
        # Pozycje jointów spoza pary az/el (gdyby kontroler miał ich więcej)
        # zostawiamy tam, gdzie są - nie wymyślamy im wartości.
        msg.data = [float(by_name.get(j, self._positions.get(j, 0.0))) for j in self._order]
        self._pub_cmd.publish(msg)

    def _cb_raw(self, msg: Vector3Stamped):
        # z = moment WYSŁANIA żądania pomiaru (patrz nagłówek).
        self._last_shot_start = msg.vector.z

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait_fresh(self, t_gate: float) -> bool:
        """Czeka na pomiar rozpoczęty PO t_gate. False = kazano się zatrzymać."""
        deadline = self._now() + self._max_wait
        while self._last_shot_start <= t_gate:
            if self._now() > deadline:
                # Cel, którego dalmierz nie umie zmierzyć w rozsądnym czasie -
                # jedziemy dalej, punkt zostanie bez odczytu. To właściwy wynik:
                # brak pomiaru jest uczciwszy niż liczba wzięta z przypadku.
                self._n_wait_timeout += 1
                self.get_logger().warn(
                    f"Brak pomiaru w {self._max_wait:.1f} s - jadę dalej "
                    f"({self._n_wait_timeout} takich punktów)",
                    throttle_duration_sec=10.0)
                return True
            if not self._sleep(0.02):
                return False
        return True

    def _set_collecting(self, value: bool):
        self._pub_collecting.publish(Bool(data=value))

    def _sleep(self, seconds: float) -> bool:
        """Przerywalny sleep. Zwraca False, gdy w międzyczasie kazano się zatrzymać."""
        return not self._stop.wait(seconds)

    def _goto(self, az_deg, el_deg) -> bool:
        az_rad, el_rad = math.radians(az_deg), math.radians(el_deg)
        self._send(az_rad, el_rad)

        deadline = self.get_clock().now().nanoseconds * 1e-9 + self._timeout
        while not self._at_target(az_rad, el_rad):
            if not self._sleep(0.01):
                return False
            if self.get_clock().now().nanoseconds * 1e-9 > deadline:
                self.get_logger().warn(
                    f"Brak dojazdu do (az={az_deg:.1f}, el={el_deg:.1f}) w {self._timeout:.1f} s "
                    f"- jadę dalej. Sprawdź limity w EEPROM serwa albo profil prędkości.",
                    throttle_duration_sec=5.0)
                return True
        return True

    # --------------------------------------------------------- trapez prędkości

    @staticmethod
    def _profile_time(distance_deg, vmax, accel):
        """Czas trwania trapezu (albo trójkąta, gdy droga za krótka na rozpęd)."""
        d = abs(distance_deg)
        if d < 1e-9:
            return 0.0
        if d <= vmax * vmax / accel:            # trójkąt: nie zdąży rozpędzić
            return 2.0 * math.sqrt(d / accel)
        return 2.0 * vmax / accel + (d - vmax * vmax / accel) / vmax

    def _move(self, az_to, el_to, vmax) -> bool:
        """Przejazd obu osi do (az_to, el_to) PROFILEM SERWA.

        Prędkość i przyspieszenie rozkładamy na osie proporcjonalnie do drogi,
        żeby obie skończyły razem - ruch idzie po prostej w przestrzeni jointów.
        Oś bez drogi dostaje pełną wartość: dla niej to bez znaczenia, a surowe
        0 znaczyłoby "bez profilu", czyli pełną prędkość.
        """
        az_from = math.degrees(self._positions.get(self._az_joint, 0.0))
        el_from = math.degrees(self._positions.get(self._el_joint, 0.0))
        d_az, d_el = az_to - az_from, el_to - el_from
        dist = math.hypot(d_az, d_el)
        if dist < 1e-3:
            return True

        def share(d):
            return abs(d) / dist if abs(d) > 1e-3 else 1.0

        vel = {self._az_joint: vmax * share(d_az), self._el_joint: vmax * share(d_el)}
        acc = {self._az_joint: self._accel * share(d_az), self._el_joint: self._accel * share(d_el)}
        for pub, values, fallback in ((self._pub_pvel, vel, vmax), (self._pub_pacc, acc, self._accel)):
            msg = Float64MultiArray()
            msg.data = [float(values.get(j, fallback)) for j in self._order]
            pub.publish(msg)
        # Profil musi dojść do serwa PRZED celem: serwo planuje trajektorię
        # w chwili przyjęcia Goal Position. Trzy cykle pętli z zapasem.
        if not self._sleep(0.08):
            return False
        self._send(math.radians(az_to), math.radians(el_to))

        expected = self._profile_time(dist, vmax, self._accel)
        deadline = self._now() + expected * 1.5 + 2.0
        while not self._at_target(math.radians(az_to), math.radians(el_to)):
            if not self._sleep(0.02):
                return False
            if self._now() > deadline:
                self.get_logger().warn(
                    f"Brak dojazdu do (az={az_to:.1f}, el={el_to:.1f}) w {expected * 1.5 + 2.0:.1f} s "
                    f"- jadę dalej. Sprawdź limity w EEPROM serwa (Sync Write nie zgłasza "
                    f"celu spoza okna).", throttle_duration_sec=5.0)
                return True
        return True

    # --------------------------------------------------------- sweep ciągły

    def _run_continuous(self):
        rows = self._rows
        self.get_logger().info("Sweep ciągły start.")
        done = 0
        for az_deg, el_from, el_to in rows:
            if self._stop.is_set():
                break

            # Dojazd na początek wiersza jest PRZEJAZDEM NIEZBIERAJĄCYM: bramka
            # zamknięta, prędkość transportowa.
            self._set_collecting(False)
            if not self._move(az_deg, el_from, self._travel_speed):
                break
            # Chwila na uspokojenie konstrukcji po zatrzymaniu, zanim ruszy
            # zbierający przejazd - inaczej pierwsze punkty wiersza łapią drgania.
            if not self._sleep(self._settle):
                break

            self._set_collecting(True)
            ok = self._move(az_deg, el_to, self._sweep_speed)
            self._set_collecting(False)
            if not ok:
                break

            done += 1
            self.get_logger().info(f"Wiersz {done}/{len(rows)} (az={az_deg:.1f} st) gotowy")

        if self._return_to_zero and not self._stop.is_set():
            self.get_logger().info("Powrót do zera.")
            self._move(0.0, 0.0, self._travel_speed)

        state = "done" if not self._stop.is_set() else "aborted"
        self._pub_state.publish(String(data=state))
        self.get_logger().info(f"Sweep ciągły zakończony ({done}/{len(rows)} wierszy).")

    # ------------------------------------------------------------------- sweep

    def _run(self):
        # Kontroler musi wstać i podać pierwsze /joint_states, zanim zaczniemy
        # cokolwiek liczyć na temat dojazdu.
        while not self._positions and self._sleep(0.1):
            pass
        if self._stop.is_set():
            return

        self._set_collecting(False)
        self._pub_state.publish(String(data="running"))

        if self._mode == "continuous":
            self._run_continuous()
            return

        self.get_logger().info("Sweep start.")

        done = 0
        for az_deg, el_deg in self._points:
            if self._stop.is_set():
                break

            self._set_collecting(False)
            if not self._goto(az_deg, el_deg):
                break

            # Osobno dojazd i uspokojenie: po dotarciu w tolerancję konstrukcja
            # jeszcze drga, a Sharp uśrednia okno wstecz - zbieranie od razu
            # zaciągnęłoby ogon poprzedniej pozycji.
            if not self._sleep(self._settle):
                break

            t_gate = self._now()
            self._set_collecting(True)

            if self._wait_meas:
                # Głowica rusza dopiero, gdy wróci pomiar rozpoczęty po t_gate.
                if not self._wait_fresh(t_gate):
                    break
            if self._dwell > 0.0 and not self._sleep(self._dwell):
                break
            self._set_collecting(False)

            done += 1
            if done % 25 == 0 or done == len(self._points):
                self.get_logger().info(f"Postęp: {done}/{len(self._points)} punktów")

        self._set_collecting(False)

        if self._return_to_zero and not self._stop.is_set():
            self.get_logger().info("Powrót do zera.")
            self._goto(0.0, 0.0)

        state = "done" if not self._stop.is_set() else "aborted"
        self._pub_state.publish(String(data=state))
        self.get_logger().info(
            f"Sweep zakończony ({done}/{len(self._points)} punktów, "
            f"{self._n_wait_timeout} bez pomiaru w limicie).")

    def stop(self):
        self._stop.set()


def main():
    rclpy.init()
    node = SphericalSweepNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
