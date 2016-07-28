# tango-live-groundplan
An Android Application for Google's Tango devices based on their java-floorplan-example. 
It scans the surroundings automatically and generates a groundplan for the current location, once finished.

## Setup
Clone this repository to your machine (e.g. `C://Android/Projects/tango-groundplan`). 
Then, clone [the official java examples](https://github.com/googlesamples/tango-examples-java) at release **MIRA** and copy `java_examples_utils` and `TangoReleaseLibs` to the **same** directory (e.g. `C://Android/Projects/`).
Finally, import the project with Android Studio.

## Usage
Start the app, wait for about 10 seconds, tap on "Measure", and start moving the device slowly through your location. 
If it recognizes a room, it is highlighed and displayed in AR. Make sure to scan the walls in clockwise direction (move to your right).
Once you're done, hit "Done" and the app will show you a groundplan of your location. 
Measurements in good conditions have been around 98% accurate.

## Limits
Beware that moving the device too fast will lead to a crash of Tango and therefore the app.
Very bright areas with direct sunlight can be difficult. As may be windows, reflective surfaces etc.
