#!/usr/bin/env python3
"""3D models for every part that has a body (run with KiCad's Python; safe to rerun at any time):

    flatpak run --command=python3 org.kicad.KiCad kicad/rev3/gen/pcb_models.py

KiCad 10's 3D library has no model for several of its own footprints (AMASS XT30UPB, Molex CLIK-Mate,
2920 fuse, two package variants, ESP32-S3-WROOM-1U), and the custom footprints had none.
- Custom footprints get their model in PowerBoard_rev3.pretty (so the library and the board agree).
- Library footprints whose model file does not exist get a replacement on the board instance.
Connector models are rev2's STEP files in kicad/ext (same footprints, same offsets as rev2's board).
Stand-ins (scaled or neighbouring packages) are marked in STAND_IN; bare pads, test points, holes
and wire bridges have no body and no model.
"""
import os
import pcbnew

PROJ = "kicad/rev3"
LIB = f"{PROJ}/PowerBoard_rev3.pretty"
EXT = "${KIPRJMOD}/../ext/SamacSys_Parts.3dshapes"
K3D = "${KICAD10_3DMODEL_DIR}"
K3D_PATH = "/app/extensions/Library/3dmodels"

# name -> (model, offset xyz, rotation xyz, scale xyz)
XT90 = ((26, 5, 0), (-90, 0, 180))                 # rev2 board's placement of the SamacSys XT90 models
DEANS = ((-0.7, 0, 11), (90, 180, 90))             # rev2 board's placement of the Deans models
LIB_MODELS = {
    "XT90PB-F": (f"{EXT}/XT90PB-F.stp", *XT90, (1, 1, 1)),
    "XT90PB-M": (f"{EXT}/XT90PB-M.stp", (5, 0, 0), (0, 0, 180), (1, 1, 1)),       # rev2's battery-input placement
    "DEANS-T": (f"{EXT}/T-Plug female - W.S Deans Co. Est. 1957.step", *DEANS, (1, 1, 1)),
    "R_Bourns_CSS2H-2512_Kelvin": (f"{K3D}/Resistor_SMD.3dshapes/R_2512_6332Metric.step", (0, 0, 0), (0, 0, 0), (1, 1, 1)),
    "TI_DGX0019A_VSSOP-19": (f"{K3D}/Package_SO.3dshapes/MSOP-16_3x4.039mm_P0.5mm.step", (0, 0, 0), (0, 0, 0),
                             (1, 5.1 / 4.039, 1)),
    "Fuse_Bourns_MF-R300_P5.08mm": (f"{K3D}/Capacitor_THT.3dshapes/C_Disc_D7.5mm_W2.5mm_P5.00mm.step", (0, 0, 0),
                                    (0, 0, 0), (5.08 / 5.0, 1, 1)),
    "ESP32-S3-WROOM-1U_HandSolder": (f"{K3D}/RF_Module.3dshapes/ESP32-WROOM-32U.step", (0, 0, 0), (0, 0, 0), (1, 1, 1)),
}
# missing library model (file name) -> replacement
REPLACE = {
    "AMASS_XT30UPB-F_1x02_P5.0mm_Vertical.step": (f"{EXT}/Amass_XT30UPB-F.step", (2.5, 0, 0), (0, 0, 0), (1, 1, 1)),
    "AMASS_XT30UPB-M_1x02_P5.0mm_Vertical.step": (f"{EXT}/Amass_XT30UPB-M.step", (2.5, 0, 0), (0, 0, 0), (1, 1, 1)),
    "Molex_CLIK-Mate_502585-0470_1x04-1MP_P1.50mm_Horizontal.step":
        (f"{K3D}/Connector_Molex.3dshapes/Molex_PicoBlade_53261-0471_1x04-1MP_P1.25mm_Horizontal.step",
         (0, 0, 0), (0, 0, 0), (1.5 / 1.25, 1, 1)),
    "Fuse_2920_7451Metric.step": (f"{K3D}/Resistor_SMD.3dshapes/R_2816_7142Metric.step", (0, 0, 0), (0, 0, 0),
                                  (7.45 / 7.1, 5.45 / 4.2, 1)),
    "HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm_Mask2.96x2.96mm.step":
        (f"{K3D}/Package_SO.3dshapes/HTSSOP-20-1EP_4.4x6.5mm_P0.65mm_EP3.4x6.5mm.step", (0, 0, 0), (0, 0, 0), (1, 1, 1)),
    "SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.3mm.step":
        (f"{K3D}/Package_SO.3dshapes/SOIC-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.81mm.step", (0, 0, 0), (0, 0, 0), (1, 1, 1)),
}
MALE_DEANS = LIB_MODELS["DEANS-T"]      # like rev2: the male STEP has no known placement, the female body stands in
STAND_IN = ("CSS2H (2512 body)", "VSSOP-19 (stretched MSOP-16)", "ESP32-S3-WROOM-1U (WROOM-32U body)",
            "CLIK-Mate (stretched PicoBlade)", "2920 PTC (scaled 2816)")


def model(spec):
    path, off, rot, scale = spec
    m = pcbnew.FP_3DMODEL()
    m.m_Filename = path
    m.m_Offset = pcbnew.VECTOR3D(*off)
    m.m_Rotation = pcbnew.VECTOR3D(*rot)
    m.m_Scale = pcbnew.VECTOR3D(*scale)
    return m


def set_model(fp, spec):
    fp.Models().clear()
    fp.Models().push_back(model(spec))


def exists(path):
    p = path.replace(K3D, K3D_PATH).replace("${KIPRJMOD}", PROJ)
    return os.path.exists(p)


for spec in list(LIB_MODELS.values()) + list(REPLACE.values()) + [MALE_DEANS]:
    assert exists(spec[0]), spec[0]

io = pcbnew.PCB_IO_KICAD_SEXPR()
for name, spec in LIB_MODELS.items():
    fp = io.FootprintLoad(LIB, name)
    set_model(fp, spec)
    io.FootprintSave(LIB, fp)

board = pcbnew.LoadBoard(f"{PROJ}/PowerBoard_rev3.kicad_pcb")
done = {}
for fp in board.GetFootprints():
    name = fp.GetFPID().GetLibItemName().wx_str()
    if name in LIB_MODELS:
        spec = MALE_DEANS if name == "DEANS-T" and "male (" in fp.GetValue() else LIB_MODELS[name]
        set_model(fp, spec)
        done[name] = done.get(name, 0) + 1
        continue
    for m in list(fp.Models()):
        if not exists(m.m_Filename):
            base = os.path.basename(m.m_Filename)
            if base in REPLACE:
                set_model(fp, REPLACE[base])
                done[base] = done.get(base, 0) + 1
            else:
                print("still missing:", fp.GetReference(), m.m_Filename)
board.Save(f"{PROJ}/PowerBoard_rev3.kicad_pcb")
for k, v in sorted(done.items()):
    print(f"{v:3d} x {k}")
