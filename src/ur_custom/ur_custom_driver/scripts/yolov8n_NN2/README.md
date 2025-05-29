## Folder description
This entire folder in the output of the trained YOLOv8 model, is does not contain any file to train the model neither the dataset to do so. Is just a copy-paste of the model obtained after such a training process.

## Model description
The model takes an BGR image as input and returns a value from 0 to 9, alongside to the 4 coordinates of the bounding box encapturing the idenfied object in the image.

If the model detects more than one object, it will return more than one set of values.

The meaning of the values are explained as follows:

0 - Drone Empty
1 - Drone Full
2 - Station Top Right Empty
3 - Station Top Right Full
4 - Station Top Left Empty
5 - Station Top Left Full
6 - Station Bottom Right Empty
7 - Station Bottom Right Full
8 - Station Bottom Left Empty
9 - Station Bottom Left Full