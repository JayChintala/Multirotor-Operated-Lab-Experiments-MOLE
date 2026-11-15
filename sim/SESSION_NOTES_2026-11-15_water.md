# Session notes: 2026-11-15 (continued), water in the module

## Done
- **Water added as its own link** inside the water module (`loads` in `config/vehicle.json`; fixed to the drone for now, ready to become a slosh mass). Team: up to 500 ml in flight; approximated as filling the bottom half of the module's box.
  - Mismatch: that half box (96 × 186 × 47.5 mm) holds ~848 ml, so the 500 g is spread over it. Use the real tank's inside dimensions when known.
  - Switch per run: `python3 scripts/gen_model.py --load water=0` (empty) or `--load water=0.5` (full), then restart the simulation.
- Module shown see-through, with the water as a blue block inside it.

## Results (Gazebo GUI, MOT_SPIN_MAX 0.73)
| Gains (× real FC) | Module empty (2.734 kg) | Module + 500 ml (3.234 kg) |
|---|---|---|
| ×0.6 (suggested) | steady | steady, 11/11 |
| ×1.0 | steady | steady |
| ×1.1 | steady | not tested |
| ×1.25 | oscillates (71 °/s) | steady |
| ×1.5 | oscillates | oscillates (72 °/s) |

- Hover output: empty 0.394, full 0.433 (model predicted 0.395 / 0.433).
- Thrust/weight at the 0.73 limit: empty 2.9, full 2.5.
- The empty case limits the tune; ×0.6 works for both with ~2× margin.

## Sloshing: plan
- Gazebo has no fluid simulation. Standard approach: equivalent mechanical model (fixed mass + slosh mass on a spring/pendulum tuned to the tank's first sloshing mode).
- First-mode estimates for the half-full box: **~1.7 Hz side to side**, **~2.7 Hz front to back**: in the attitude controller's range, so worth modelling.
- Needs: the real tank's inside dimensions and fill level.

## Not committed yet
Water load, see-through module, these notes. (Earlier changes from today: see the git commands given in chat.)
