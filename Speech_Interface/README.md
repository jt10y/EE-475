# ADA_Talk

* **Switch to finalDemo branch for Summer 2019 demo!**
* **Publishes ROS topics the message received from Alexa. Needs further ada_demos package integration to adapt the published msgs.**
* **ada_demos branch: tao/summer2019 has already adapt the published topics to interace with the demo. Consult Ethan for changes made since Tao's leaving**

Speech Interface for ADA DEMO Summer 2019

The Alexa skill can be accessed through: https://developer.amazon.com/alexa/console/ask, the one for ADA Summer 2019 Demo is called FeedingDemo.

## Hardware Requirement
Amazon Echo Dot (Alexa) needs to be powered from power source of your choice.

## How to start
* **The following commands should be executed in this package folder**

```
roslaunch rosbridge_server rosbridge_websocket
bst proxy lambda index.js
```

## Usage
The default invocation is: Alexa, ask the robot.

The invocation for asking food items is: Alexa, ask the robot to bring me (the) {FOOD_ITEM}.

The invocation for acquisition action choice is: Alexa, ask the robot to use {ACTION}.

The invocation for bringing food to person after acquisition is: Alexa, ask the robot to bring me (the) {FOOD}

The invocation for feeding person with an angle is: Alexa, ask the robot to feed me with/with an {ANGLE} (This does not perform very well. May need multiple invocatio to get it accurately understand. Consider changing the way of invocation)

## Change the invocation/setting of Alexa
* **Any of the above invocations can be changed through the Alexa Developer Console with the personalroboticsstudies@gmail.com account.**

After your change to the skill. You need to copy the content in the JSON Editor to ADA_Talk/models/en-US.json. This file will be used for skill interaction since we're proxying the Alexa service to our local machine.

Also remember to update the Endpoint in the Alexa Developer Console if you made changes to LOVELACE bst service.

Currently, the endpoint is: https://aspiring-huxley.bespoken.link. You can verify it against the prompt after running the ```bst proxy lambda index.js``` command.
