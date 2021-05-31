# Changelog

## Version 1.2.0: Final demo version
- Avoided replanning during the cooldown even if another image is processed by the AI.
- Selectable AI image no process or cooldown time using the environment variable `IMG_ENABLE_WAIT_TIME` (default `120`)
- Corrected the robot prefixes in order to work with the new fms scheme
- Removed single node selection, now it accept multiple nodes
- Fixed the hardcoded nodes, now the nodes to block are selected by environment variable `NODES_SELECTED` (default `203 403`)

## Version 1.1.0: Real robot test
- Working with real robot
- TODO: added and hardcoded node 7

## Version 1.0.0
- First release