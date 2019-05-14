# _C.A.R.L.O.S_

## The goal of the microservices

Carlos, our vehicle, is supposed to handle an intersection. More specifically, Carlos will approach an intersection which has another vehicle in the North and East lanes.

1. Carlos should stop safely before the intersection.
2. Carlos should register its position in the queue of vehicle at the intersection
3. Carlos should decide on where to drive once its turn arises
4. Carlos should drive safely in and out of the intersection

## The micro-services

1. **[ACC](./control/acc/src/acc.cpp)**: Adaptive Cruise Control (or acc for short) adjusts the vehciles speed to maintain a safe distances from surround vehicles.

2. **[CMD](./control/cmd/src/cmd.cpp)**: Command (or CMD for short) is responsible for prompting the user to pick the path that the vehcile should take (East, West or North lane). While this service can be used arbitrarily, it is most useful when the vehicle is about to leave the intersection.

3. **[Color](./vision/color_detection/src/color.cpp)**: The Color micro-service detects objects according to certain colors (e.g. green = car) and acts accordingly. The color detection services provides the area of the color detect as well as the position of the color according to the frame _(these values can potentially lead to an alternative ACC in the future)_.
   The Color services is also responsible for counting the vehicles at the intersection and tracking them accordingly.

4. **[Object](./vision/object_detection/src/object-detection.cpp)**: The object detection detects road signs. This allows us to approach the intersection contextually. Additionally, since there is no way to detect lanes and intersections in general, the object micro-service acts as the closest form of object detection because according to our assumptions, every intersection is accompined with a sign, hence by recognizing signs, the object detection services also detects intersections.

5. **[Delegator](./delegatore/src/delegator.cpp)**: The delegator is responsible for coordinating the other micro-services. The delgator is also responible for managing the data that flows between the micro-services, adjusting the "STAGES" (more on this later) and acting as a general form of tie breaker/ prioritizer that manages data results according to the most relevant information (e.g. if the ACC services alerts that there is a probable collision and the Color detection assumes that the car {represented by a color} is at a safe distance, the delegator will prioritze the ACC service over the Color because of the tangible aspect of the ACC and glitchy issues that occur with the Color service).

## The backbone of the micro-services

All the micro-services should be able to execute their responsibilities irregardless of the other micro-services. This illustrates reusablity and maintainability. However, when they **do** work together, the tasks they are executing can be delgated more efficiently. This helps avoid "_over-commanding_" the vehicle from all the different micro-services that talk to it.
