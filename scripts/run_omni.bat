@echo off
timeout 2
echo Running rotate
start cmd /k python3.11 omni_sim.py 5

echo Running circle
start cmd /k python3.11 omni_sim.py 4

echo Running circle + rotate
start cmd /k python3.11 omni_sim.py 6
