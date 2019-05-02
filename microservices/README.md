# README for Microservices

The list of micro services categories and what they do:

1. **delegator**:

   - communicates both, with the car and other microservices. the delegator will also have to delegate data flow.

2. control:

   - **acc** helps vehicle avoid obstacles
   - **cmd** instructs vehicle to turn

3. vision:

   - **color_detection** detects signs and the vehicle in front of carlos
   - **object_detection** detects cars in intersection
