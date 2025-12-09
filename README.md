A geospatial Vehicle Routing Problem (VRP) optimization system built in Python using OSRM, Google OR-Tools, and real customer order data.
This engine computes real-world travel distances, applies capacity constraints, generates optimal delivery routes, and even simulates multi-trip vehicle operations for logistics planning
This project was developed as a routing optimizer for a fruit/Vegetable devilering app, a hyperlocal delivery operation.
It solves the Vehicle Routing Problem using:
	•	Real distances from a local OSRM server
	•	Capacity-constrained VRP using OR-Tools
	•	Automated customer filtering to remove invalid/non-Bangalore coordinates
	•	Multi-trip delivery simulation when demand exceeds fleet capacity
	•	Detailed route outputs with customer names, city, order value, stops & distance
	•	CSV-driven inputs for easy operational integration
