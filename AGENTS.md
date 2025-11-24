## Project ##
This project is a AI thinker ESP32 CAM project using PlatformIO and Arduino framework
Using C++ for logic, agent generated code should be C++
The project purpose is to control a winter cat shelter
- control the relay to activate a heated blanket when cold outside and cat present
- take a picture and upload to an S3 bucket every 60 minutes or whenever PIR sensor detects presence, but not more than once every 5 min
- turn off WIFI when idle

## Engineering ##
This is a hobby project with only one deployment. Do not overengineer. Is more im portant to keep the code easy to udnerstand and maintain.
All testing is manual. Do not plan to testing or acceptance, do not write tests. Just ask the user to verify the changes.
Always confirm with the user that the issue you work on is completed and verified.
When resolving an issue, commit. Never push because push may trigger Github Actions and consume free project actions time quota. the user will push manually when needed.

### Hardware ###
There are 3 devices connected
- GPIO12 connects to a relay control (output). When HIGH it will activate a termal blanket
- GPIO13 is connected to a PIR HC-SR501 sensor (input). It detecs cat(s) presence
- GPIO14 is connected to a DHT22/AM2302 temperature/humidity senzor.
The onboard camera and wifi are functional and used by the project. The SD card is not enabled.


## Coding ##
- do not attempt to upload the firmware yourself. The hardware requires me to change wiring to put it in boot mode before upload
- **IMPORTANT** do not store secrets and credentials in code. The repo is public. Use indirection methods like .h file excluded from git, environment variables, .env files etc
- if you need to generate temporary or debug files, do not place them in the project folder. Use %TMP%

