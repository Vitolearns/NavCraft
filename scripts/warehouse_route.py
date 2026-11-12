"""Map-frame approach points; goals stay outside rack/table collision volumes."""
STATIONS = [
    # Give the rotating footprint clearance from the low payload box at receiving.
    ('receiving', (-3.2, 0.4)),
    ('storage_b', (-2.0, 3.0)),
    ('storage_a', (1.5, 2.6)),
    ('packing', (3.2, -2.0)),
    ('dock_approach', (-3.2, -3.2)),
    ('home', (0.0, 0.0)),
]
ROUTES = {'smoke': [(-1.5, 0.0), (0.0, 0.0)],
          'warehouse': [point for _, point in STATIONS]}
