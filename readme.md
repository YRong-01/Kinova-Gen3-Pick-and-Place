A codebase based on Kortex API, for kinova Gen3 7-dof robot arm.

## Installation

Install the kortex API
`python3 -m pip install kortex_api-2.7.0.post5-py3-none-any.whl`

Other dependencies:

`python environment 3.10/3.11.13/3.12/3.13`

## Running 

Run with castesian points
`python pick_and_place_points.py`

Run with castesian waypoints (using a different API)
`python pick_and_place_points.py`

For now, the speed of robot using waypoint approach is significantly faster than using points. We are currently looking for solutions