# Speech Interface for Smart Wheelchair

* **This repo is still under heavy development. README will update as needed

* **Publishes ROS topics the message received from Alexa. Needs further ROS package integration to adapt the published msgs.**


The Alexa skill can be accessed through: https://developer.amazon.com/alexa/console/ask, the one for the Capstone Demo is called Smart Wheelchair.

## Hardware Requirement
Amazon Echo (Alexa) needs to be powered from power source of your choice.

## How to start
* **The following commands should be executed in this package folder**

```
roslaunch rosbridge_server rosbridge_websocket
bst proxy lambda index.js
```

## Usage
The default invocation is: Alexa, ask the wheelchair.

The invocation for asking food items is: Alexa, ask the robot to bring me (the) {FOOD_ITEM}.

The invocation for acquisition action choice is: Alexa, ask the robot to use {ACTION}.

The invocation for bringing food to person after acquisition is: Alexa, ask the robot to bring me (the) {FOOD}

The invocation for feeding person with an angle is: Alexa, ask the robot to feed me with/with an {ANGLE} (This does not perform very well. May need multiple invocatio to get it accurately understand. Consider changing the way of invocation)

## Change the invocation/setting of Alexa
* **Any of the above invocations can be changed through the Alexa Developer Console with the Tao's account. Let him know if you want to change anything**

After your change to the skill. You need to copy the content in the JSON Editor to Speech_Interface/models/en-US.json. This file will be used for skill interaction since we're proxying the Alexa service to our local machine.

Also remember to update the Endpoint in the Alexa Developer Console if you made changes to Raspberry Pi bst service.

Currently, the endpoint is: https://classy-albini-c6Omxj.bespoken.link. You can verify it against the prompt after running the ```bst proxy lambda index.js``` command.
