# CompRobo Final Project: Neato Soccer, Extended Edition
For this project, our ostensible goal is to program a robot (a neato vacuum cleaner) to recognize a soccer ball visually, approach it, and kick it into a goal. The real purpose is to develop our own learning goals around computer vision and object recognition via a neural network. The project is largely split along those lines.

## The Story So Far
As a part of documenting our learning during the development project, what follows are a few incremental blog posts about the paths we took as we developed the computer vision and neural network parts of the software.

### Neural Network Development

**4/18/17** - After some initial research into neural networks, we decided to start with a [multi-layered perceptron](https://en.wikipedia.org/wiki/Multilayer_perceptron) constructed using the [Lasagne package](https://github.com/Lasagne/Lasagne), closely following in the footsteps of a [blog post](http://danielnouri.org/notes/2014/12/17/using-convolutional-neural-nets-to-detect-facial-keypoints-tutorial/) by Daniel Nouri.

A multi-layered perceptron (MLP) is more or less the simplest version of a neural network. It contains many independent nodes, organized into sequential layers, that are activated or not activated by some combination of the input node values. It's not a great match for learning from images, because, by default, it lacks any concept of how an image is structured; that is, by making all of the input nodes (pixel values) independent, each has no relation to its adjacent pixels unless that relation is 'learned' by extensive training. We started here, however, because it is simple to implement.

To get the MLP running, we quickly built a system to crop, grayscale, and down-sample images to 128x128 pixel squares. We fed 20 images through this system and into the MLP. Of the images 8 contained a target object and were tagged and 12 did not contain an image and were untagged. From these twenty images, our MLP attempts to determine what differentiates a tagged image from an untagged image. Our MLP randomly selects 4 images from the set to test on after training and the results from those tests are quite poor. Almost independent of whether the target object is *actually* in the image, the MLP has about a 1/3 confidence that the image contains the object.

There are three likely reasons this is happening. The obvious one is that 20 images does not come close to approaching the size of data sets used for typical machine learning projects - common examples of starter ML projects use about 50,000 images to train on. The second likely reason is that, as described, an MLP does a poor job of understanding the 2D structure of an image. A convolutional neural network - a variant that does account for 2D structure - may do a better job and require less training to achieve reasonable results. A third possible reason is simply that our MLP is structured poorly. At present, the 128x128 input pixel values are fed into 100 hidden nodes, and then to 1 output node. That structure is a wild guess and we'll need to do more research, test other structures, or ask for help to determine what a reasonable MLP structure for this project is.

Next, we'll be trying to address the first problem - the lack of sufficient training images - by building a system to rapidly capture and tag images from our robot's camera. That should allow us to get into the thousands of tagged ball and non-ball images that will allow us to better determine whether using an MLP is a tenable solution to object recognition, or if we need a more advanced model.