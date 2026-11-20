
import math
import random
import socket
from controller import Supervisor

UDP_PORT = 5005
ACCEL = 5.0                  # secondes simulees par seconde reelle
SCEN_SIM = 300.0             # duree d'un scenario (s simulees) -> 60 s reelles
SCEN_WALL = SCEN_SIM / ACCEL
FAN_MAX_RAD = 30.0           # rad/s a PWM = 1


# ---------------- scenarios (vitesse en km/h) ----------------
def v_bouchon(t):
    return max(0.0, 18.0 * math.sin(2 * math.pi * t / 110.0))


SCEN = [
    dict(name="Embouteillage",   amb=32.0, load=0.35, rain=0.0, sky=(0.60, 0.66, 0.74), sun=1.0,
         fog=600.0, fog_col=(0.80, 0.82, 0.85), disk=False, traffic=True,  speed=v_bouchon),
    dict(name="Autoroute",       amb=28.0, load=0.60, rain=0.0, sky=(0.42, 0.62, 0.95), sun=1.3,
         fog=600.0, fog_col=(0.80, 0.85, 0.95), disk=False, traffic=False, speed=lambda t: 120.0),
    dict(name="Pluie",           amb=16.0, load=0.45, rain=1.0, sky=(0.36, 0.38, 0.42), sun=0.45,
         fog=60.0,  fog_col=(0.50, 0.52, 0.55), disk=False, traffic=False, speed=lambda t: 80.0),
    dict(name="Ete caniculaire", amb=43.0, load=0.72, rain=0.0, sky=(0.55, 0.75, 1.00), sun=1.8,
         fog=220.0, fog_col=(0.95, 0.90, 0.78), disk=True,  traffic=False, speed=lambda t: 90.0),
]
FAULT_WINDOW = (100.0, 250.0)   # panne ventilateur en demo : scenario 4, t local (s simulees)
DECOR = ["Ville", "Autoroute", "Campagne sous la pluie", "Zone desertique, palmiers"]
ENV_COL = [
    dict(ground=(0.36, 0.36, 0.38), hill=(0.50, 0.50, 0.55), crown=(0.20, 0.45, 0.20)),
    dict(ground=(0.24, 0.46, 0.20), hill=(0.30, 0.42, 0.30), crown=(0.15, 0.45, 0.15)),
    dict(ground=(0.12, 0.28, 0.12), hill=(0.18, 0.26, 0.20), crown=(0.07, 0.26, 0.09)),
    dict(ground=(0.80, 0.68, 0.42), hill=(0.72, 0.56, 0.36), crown=(0.20, 0.45, 0.20)),
]

# ---------------- portage Python du modele Simulink (demo) ----------------
P = dict(Cm=30000.0, Cl=25000.0, Cr=10000.0, Kmc=2000.0, Kcr=2500.0, UA0=40.0, Kv=2.0, Kfan=150.0,
         Pfan=350.0, Qbase=4000.0, Qload=20000.0, Tref=90.0, Ttherm0=75.0, Ttherm1=90.0,
         Kp=0.2, Ki=0.005, Kaw=0.1, Tm0=80.0, Tl0=75.0, Tr0=65.0)


class Thermal:
    def __init__(self):
        self.Tm, self.Tl, self.Tr = P["Tm0"], P["Tl0"], P["Tr0"]
        self.I = 0.0
        self.E = 0.0
        self.out = {}

    def step(self, h, vit, amb, load, pump=1, fault=0, engine=1):
        e = self.Tl - P["Tref"]
        raw = P["Kp"] * e + self.I
        lim = clamp(raw)
        pwm = lim * (1 - fault)
        thermo = clamp((self.Tl - P["Ttherm0"]) / (P["Ttherm1"] - P["Ttherm0"]))
        q_eng = (P["Qload"] * load + P["Qbase"]) * engine
        q_ml = P["Kmc"] * (self.Tm - self.Tl) * pump
        q_lr = P["Kcr"] * (self.Tl - self.Tr) * thermo * pump
        ua = P["Kv"] * vit + P["UA0"] + P["Kfan"] * pwm
        q_ra = (self.Tr - amb) * ua
        pf = P["Pfan"] * pwm ** 3
        self.out = dict(Tm=self.Tm, Tl=self.Tl, Tr=self.Tr, pwm=pwm, pf=pf, ef=self.E,
                        alarm=1 if self.Tl > 105.0 else 0, thermo=thermo, qrad=q_ra)
        self.Tm += h * (q_eng - q_ml) / P["Cm"]
        self.Tl += h * (q_ml - q_lr) / P["Cl"]
        self.Tr += h * (q_lr - q_ra) / P["Cr"]
        self.I += h * (P["Ki"] * e + P["Kaw"] * (lim - raw))
        self.E += h * pf / 3.6e6
        return self.out


def clamp(x, lo=0.0, hi=1.0):
    return max(lo, min(hi, x))


def lerp_list(cur, tgt, a):
    return [c + (t - c) * a for c, t in zip(cur, tgt)]


# ---------------- elements de la scene ----------------
N_DROPS, N_MARKS, N_TREES, N_POLES, N_TRAFFIC = 220, 24, 12, 6, 4
SPAN = 144.0
TRAFFIC_POS = [(7.0, 0.0), (14.0, 0.0), (4.5, 3.5), (11.0, 3.5)]
WHEEL_R = 0.33

robot = Supervisor()
dt_ms = int(robot.getBasicTimeStep())
dt = dt_ms / 1000.0


class _Null:                                  # champ absent (autre monde) -> no-op
    def __getattr__(self, name):
        return lambda *a, **k: None


def field(def_name, field_name):
    node = robot.getFromDef(def_name)
    return node.getField(field_name) if node else _Null()


fan_rot = field("FAN", "rotation")
engine_col = field("ENGINE_MAT", "baseColor")
rad_col = field("RAD_MAT", "baseColor")
cool_col = field("COOLANT_MAT", "baseColor")
pump_col = field("PUMP_MAT", "baseColor")
exh_glow = field("EXH_MAT", "emissiveColor")
sky_col = field("BG", "skyColor")
fog_vis = field("FOG", "visibilityRange")
fog_col = field("FOG", "color")
sun_int = field("SUN_LIGHT", "intensity")
road_rough = field("ROAD_MAT", "roughness")
head_glow = field("HEAD_MAT", "emissiveColor")
brake_glow = field("BRAKE_MAT", "emissiveColor")
tbrake_glow = field("TBRAKE_MAT", "emissiveColor")
sun_disk = field("SUN_DISK", "translation")
ground_col = field("GROUND_MAT", "baseColor")
hill_col = field("HILL_MAT", "baseColor")
crown_col = field("TREECROWN_MAT", "baseColor")
wheels = [field(f"WHEEL{i}", "rotation") for i in range(4)]
marks = [field(f"MARK{i}", "translation") for i in range(N_MARKS)]
trees = [field(f"TREE{i}", "translation") for i in range(N_TREES)]
poles = [field(f"POLE{i}", "translation") for i in range(N_POLES)]
traffic = [field(f"TRAFFIC{i}", "translation") for i in range(N_TRAFFIC)]
drops = [field(f"DROP{i}", "translation") for i in range(N_DROPS)]
blds = [field(f"BLD{i}", "translation") for i in range(12)]
pines = [field(f"PINE{i}", "translation") for i in range(12)]
signs = [field(f"SIGN{i}", "translation") for i in range(2)]
palms = [field(f"PALM{i}", "translation") for i in range(8)]
bushes = [field(f"BUSH{i}", "translation") for i in range(12)]
envs0 = field("ENVS0", "translation")
envs1 = field("ENVS1", "translation")

mark_x0 = [-72.0 + i * 6.0 for i in range(N_MARKS)]
tree_x0 = [-72.0 + i * 12.0 for i in range(N_TREES)]
pole_x0 = [-72.0 + i * 24.0 for i in range(N_POLES)]
bld_x0 = [-72.0 + i * 12.0 for i in range(12)]
pine_x0 = [-66.0 + i * 12.0 for i in range(12)]
sign_x0 = [-36.0, 36.0]
palm_x0 = [-72.0 + i * 18.0 for i in range(8)]
bush_x0 = [-67.0 + i * 12.0 for i in range(12)]
random.seed(3)
drop_pos = [[random.uniform(-8, 10), random.uniform(-6, 8), random.uniform(0, 6)] for _ in range(N_DROPS)]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", UDP_PORT))     # 0.0.0.0 : accepte aussi les paquets venant de WSL
sock.setblocking(False)

shown = {}


def place(name, i, fld, x, y, g):
    """Element du decor : 'pousse' du sol quand g -> 1, disparait quand g -> 0."""
    key = (name, i)
    if g < 0.03:
        if shown.get(key, True):
            fld.setSFVec3f([0.0, y, -100.0])
            shown[key] = False
        return
    shown[key] = True
    fld.setSFVec3f([x, y, -(1.0 - g) * 40.0])


def wrap(x0, d):
    return ((x0 - d + 72.0) % SPAN) - 72.0


# ---------------- etat ----------------
model = Thermal()
M = dict(Tm=P["Tm0"], Tl=P["Tl0"], Tr=P["Tr0"], pwm=0.0, pf=0.0, ef=0.0, alarm=0, thermo=0.0, qrad=0.0,
         pump=1, fault=0)
env_g = [1.0, 0.0, 0.0, 0.0]
ground_c, hill_c, crown_c = list(ENV_COL[0]["ground"]), list(ENV_COL[0]["hill"]), list(ENV_COL[0]["crown"])
fan_angle, fan_omega, wheel_angle, dist = 0.0, 0.0, 0.0, 0.0
v = v_prev = 0.0                                   # m/s
amb, rain = SCEN[0]["amb"], 0.0
sky, fogc, fogv, sun = list(SCEN[0]["sky"]), list(SCEN[0]["fog_col"]), SCEN[0]["fog"], SCEN[0]["sun"]
rough = 0.95
last_rx = -10.0
ext = dict(scen=0, vit=0.0, amb=SCEN[0]["amb"], t=0.0)
active_drops = 0

while robot.step(dt_ms) != -1:
    now = robot.getTime()

    # ---------- UDP : paquets "M,..." (MATLAB / ROS 2) ----------
    try:
        while True:
            data, _ = sock.recvfrom(512)
            try:
                p = data.decode().strip().split(",")
                if p[0] == "M" and len(p) >= 18:
                    x = [float(c) for c in p[1:18]]
                    M.update(Tm=x[1], Tl=x[2], Tr=x[3], pwm=x[4], pf=x[8], ef=x[9], alarm=int(x[10]),
                             thermo=x[11], qrad=x[12], pump=int(x[14]), fault=int(x[15]))
                    ext.update(t=x[0], amb=x[5], vit=x[6], scen=int(x[13]) - 1)
                    last_rx = now
            except (ValueError, IndexError):
                pass
    except BlockingIOError:
        pass
    external = (now - last_rx) < 1.0

    # ---------- scenario courant ----------
    if external:
        idx = max(0, min(len(SCEN) - 1, ext["scen"]))
        sim_t = ext["t"]
    else:
        idx = int(now // SCEN_WALL) % len(SCEN)
        sim_t = (now * ACCEL) % (SCEN_SIM * len(SCEN))
    sc = SCEN[idx]
    t_loc = sim_t - idx * SCEN_SIM

    vit_kmh = ext["vit"] if external else sc["speed"](t_loc)
    amb_target = ext["amb"] if external else sc["amb"]

    # ---------- thermique : modele MATLAB recu, ou portage Python en demo ----------
    if not external:
        fault = 1 if (idx == 3 and FAULT_WINDOW[0] <= t_loc < FAULT_WINDOW[1]) else 0
        for _ in range(int(ACCEL)):                    # sous-pas -> accelere x ACCEL
            o = model.step(dt, vit_kmh, amb_target, sc["load"], 1, fault, 1)
        M.update(Tm=o["Tm"], Tl=o["Tl"], Tr=o["Tr"], pwm=o["pwm"], pf=o["pf"], ef=o["ef"],
                 alarm=o["alarm"], thermo=o["thermo"], qrad=o["qrad"], pump=1, fault=fault)

    # ---------- lissages visuels ----------
    v_prev = v
    v += (vit_kmh / 3.6 - v) * (1 - math.exp(-dt / 0.8))
    a_env = 1 - math.exp(-dt / 2.0)
    amb += (amb_target - amb) * a_env
    rain += (sc["rain"] - rain) * a_env
    sky = lerp_list(sky, sc["sky"], a_env)
    fogc = lerp_list(fogc, sc["fog_col"], a_env)
    fogv += (sc["fog"] - fogv) * a_env
    sun += (sc["sun"] - sun) * a_env
    rough += ((0.95 - 0.8 * sc["rain"]) - rough) * a_env

    # ---------- decor ----------
    a_grow = 1 - math.exp(-dt / 1.2)
    for e in range(len(env_g)):
        env_g[e] += ((1.0 if e == idx else 0.0) - env_g[e]) * a_grow
    ground_c = lerp_list(ground_c, ENV_COL[idx]["ground"], a_env)
    hill_c = lerp_list(hill_c, ENV_COL[idx]["hill"], a_env)
    crown_c = lerp_list(crown_c, ENV_COL[idx]["crown"], a_env)
    ground_col.setSFColor(ground_c)
    hill_col.setSFColor(hill_c)
    crown_col.setSFColor(crown_c)
    envs0.setSFVec3f([0.0, 0.0, -(1.0 - env_g[0]) * 40.0])
    envs1.setSFVec3f([0.0, 0.0, -(1.0 - env_g[1]) * 40.0])

    dist += v * dt
    for i in range(N_MARKS):
        marks[i].setSFVec3f([wrap(mark_x0[i], dist), 1.75, 0.006])
    g_tree_all = env_g[2]
    g_tree_even = max(env_g[1], env_g[2])
    g_pole = max(env_g[0], env_g[1], env_g[2])
    for i in range(N_TREES):
        place("tree", i, trees[i], wrap(tree_x0[i], dist), 9.0, g_tree_even if i % 2 == 0 else g_tree_all)
    for i in range(N_POLES):
        place("pole", i, poles[i], wrap(pole_x0[i], dist), 6.0, g_pole)
    for i in range(12):
        place("bld", i, blds[i], wrap(bld_x0[i], dist), 14.0, env_g[0])
        place("pine", i, pines[i], wrap(pine_x0[i], dist), 13.0, env_g[2])
        place("bush", i, bushes[i], wrap(bush_x0[i], dist), 7.4, env_g[3])
    for i in range(2):
        place("sign", i, signs[i], wrap(sign_x0[i], dist), 0.0, env_g[1])
    for i in range(8):
        place("palm", i, palms[i], wrap(palm_x0[i], dist), 8.5, env_g[3])

    # ---------- roues ----------
    wheel_angle = (wheel_angle + v / WHEEL_R * dt) % 6.2832
    for w in wheels:
        w.setSFRotation([0, 1, 0, wheel_angle])

    # ---------- ventilateur : vitesse proportionnelle au PWM du modele ----------
    fan_omega += (FAN_MAX_RAD * clamp(M["pwm"]) - fan_omega) * 0.08
    fan_angle = (fan_angle + fan_omega * dt) % 6.2832
    fan_rot.setSFRotation([1, 0, 0, fan_angle])

    # ---------- couleurs : moteur (Tm), radiateur (Tr), liquide (Tl), pompe ----------
    tm = clamp((M["Tm"] - 60.0) / 60.0)
    tr = clamp((M["Tr"] - 40.0) / 70.0)
    tl = clamp((M["Tl"] - 70.0) / 40.0)
    engine_col.setSFColor([tm, 0.35 * (1 - tm), 1 - tm])
    exh_glow.setSFColor([0.9 * tm, 0.25 * tm, 0.0])
    rad_col.setSFColor([0.08 + 0.8 * tr, 0.08 + 0.25 * tr, 0.09])
    cool_col.setSFColor([0.2 + 0.7 * tl, 0.9 - 0.7 * tl, 0.4 - 0.3 * tl])
    pump_col.setSFColor([0.1, 0.9, 0.2] if M["pump"] else [0.4, 0.4, 0.4])

    # ---------- ciel, brouillard, soleil, route ----------
    sky_col.setMFColor(0, sky)
    fog_col.setSFColor(fogc)
    fog_vis.setSFFloat(fogv)
    sun_int.setSFFloat(sun)
    road_rough.setSFFloat(rough)
    sun_disk.setSFVec3f([12.0, 45.0, 14.0 if sc["disk"] else -50.0])

    # ---------- feux ----------
    lights = clamp(rain * 1.3 + (0.6 if idx == 0 else 0.0))
    head_glow.setSFColor([lights, lights, 0.8 * lights])
    braking = v < 0.4 or (v_prev - v) / dt > 1.2
    tail = [1.0, 0.0, 0.0] if braking else [0.25, 0.0, 0.0]
    brake_glow.setSFColor(tail)
    tbrake_glow.setSFColor(tail)

    # ---------- trafic ----------
    for i in range(N_TRAFFIC):
        x, y = TRAFFIC_POS[i]
        traffic[i].setSFVec3f([x, y, 0.0 if sc["traffic"] else -30.0])

    # ---------- pluie ----------
    n_act = int(clamp(rain) * N_DROPS)
    for i in range(n_act):
        p = drop_pos[i]
        p[2] -= 12.0 * dt
        p[0] -= v * dt
        if p[2] < 0.0:
            p[0], p[1], p[2] = random.uniform(-8, 10), random.uniform(-6, 8), 6.0 + random.random()
        if p[0] < -8.0:
            p[0] += 18.0
        drops[i].setSFVec3f(p)
    for i in range(n_act, active_drops):
        drops[i].setSFVec3f([0.0, 0.0, -50.0])
    active_drops = n_act

    # ---------- affichage ----------
    robot.setLabel(0, f"Scenario : {sc['name']}  ({DECOR[idx]})", 0.02, 0.02, 0.07, 0xFFFFFF, 0.0, "Arial")
    robot.setLabel(1, f"Vitesse : {v * 3.6:5.1f} km/h     Exterieur : {amb:4.1f} C", 0.02, 0.08, 0.05, 0xCCE6FF, 0.0, "Arial")
    robot.setLabel(2, f"Moteur {M['Tm']:5.1f} C   Liquide {M['Tl']:5.1f} C   Radiateur {M['Tr']:5.1f} C", 0.02, 0.14, 0.055, 0xFFFFFF, 0.0, "Arial")
    robot.setLabel(3, f"Ventilateur PWM {M['pwm'] * 100:3.0f} %  ({M['pf']:4.0f} W, {M['ef']:.3f} kWh)   Pompe {'ON' if M['pump'] else 'OFF'}   Thermostat {M['thermo'] * 100:3.0f} %",
                   0.02, 0.20, 0.045, 0xFFD060, 0.0, "Arial")
    robot.setLabel(4, "", 0.02, 0.26, 0.04, 0x88CCFF, 0.0, "Arial")
    blink = int(now * 2) % 2 == 0
    robot.setLabel(5, "!!! ALARME SURCHAUFFE : liquide > 105 C !!!" if (M["alarm"] and blink) else "",
                   0.02, 0.32, 0.06, 0xFF2020, 0.0, "Arial")
    robot.setLabel(6, "PANNE VENTILATEUR" if M["fault"] else "", 0.02, 0.38, 0.05, 0xFF9020, 0.0, "Arial")
