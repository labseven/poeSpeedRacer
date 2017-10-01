# How to be fast
Go fast


# IR sensor sensitivity
Using a 4.7kOhm resistor on the transistor gave us 780-1024 range on analogRead.
Lower resistors had lower (950 - 1024) ranges.



# Thoughts on The Lab Handout:

## BOM:
2 IR sensors
* Orientation:
Next to each other
Using 'grey' zone to get quicker feedback on steering, with second ir sensor for fallback
One ir sensor as encoder
Both IR sensors as encoders

Build encoders using handmade switches on the axes to dead reckon more effectively.

Use IR sensor to communicate to

Alternatively, have a leading IR sensor doing edge detection to have leading knowledge of the track.
