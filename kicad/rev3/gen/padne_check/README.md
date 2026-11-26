# padne cross-check (2026-11-26)

An independent second solver for `copper_sim.py`: [padne](https://github.com/atx/padne), open-source FEM,
installed with `pipx install --system-site-packages git+https://github.com/atx/padne.git` (needs
`libgmp-dev libmpfr-dev libboost-dev libegl-dev`; the prebuilt 0.3 binary bundles KiCad 9 and cannot read this board).

- `loads.padne`: the load cases of `copper_sim.SCEN` as padne directives (same pads, same currents; copper
  conductivity 5.814e7 S/m, via plating 20 um).
- `run_padne.py PRO DIRECTIVES OUT`: solves through padne's API; a polygon its mesher rejects ("Non-manifold mesh",
  two small non-power fragments near (140, 82) on this board) is retried after a slight clean-up, logged.
- `extract.py SOLUTION BOARD`: per load case, the pads' voltages, the net's loss (sum of I * V at its pads) and
  the worst drop.

    ~/.local/share/pipx/venvs/padne/bin/python kicad/rev3/gen/padne_check/run_padne.py \
        kicad/rev3/PowerBoard_rev3.kicad_pro kicad/rev3/gen/padne_check/loads.padne /tmp/sol.padne
    ~/.local/share/pipx/venvs/padne/bin/python kicad/rev3/gen/padne_check/extract.py /tmp/sol.padne kicad/rev3/PowerBoard_rev3.kicad_pcb

padne attaches each load at one point at the pad centre, so its losses read higher than copper_sim's (which
treats a pad as one equipotential terminal: closer to a soldered tab or pin). Between points away from the pads
the two agree within about 6 % on L1, 12 % on the GND plane (see design.md).
