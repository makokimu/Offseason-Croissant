# Offseason-Croissant
Offseason code for Croissant utilizing Oblarg's Command based rewrite. Features include:
- CAN calls run concurrently using Kotlin Coroutines, completely solving loop overrun issues
- State system for joints 
- Path following using nonlinear state estimation and a nonlinear feedback controller in addition to full state drive characterization
- SolvePNP and vision code using LimeLight 2 and JeVois
- Vision integration with path following 
- Command group driven Superstructure planner to ensure safe movement
