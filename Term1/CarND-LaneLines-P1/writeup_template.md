#**Finding Lane Lines on the Road** 
Giulia Guidi

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

###1. Pipeline description

My pipeline consisted of 5 steps. 
1. First, I converted the images to grayscale
2. Then I applied a Gaussian filter
3. then the Canny edge detector was applied to filtered image
4. I then selected an area of interest in the shape of a polygon
5. Finally, the Hough transform was then applied to the resulting image with the edge detected only inside this area of interest 

In order to draw a single line on the left and right lanes, I created a draw_line_2colors method (and named this method like that because I plotted the right and the left lane with different colors).
I used a threshold on the slope values of each lines given by the Hough transform to decide is the points were belonging to the left or the right lane. 
The threshold was choosen experimentally. I tried several other options such as splitting the points depending on their x axis value (if the intersection of the 2 lanes was known, this would be the best way to process) or binning the reuslts of the slopes to separate the points into 2 categories (one for each lane) before fitting a line to those points. However, none of those methods worked to my satisfaction so I decided to keep the fixed threshold.
After separating the points into right and left, I did a linear regression on those point to fit a line.
I then backcalulated the 2 points that would be used to draw my line on the image. The idea there was to use some of the know x or y coordinated that I wanted to have on my line.
I chose to not plot any line if the list of slope values was completely abnormal (in case of a shadow on the road for instance). 

###2. Identify potential shortcomings with your current pipeline

One potential shortcoming would happen when the road roll and pitch of the car changes. The polygon used to select the area of interest for the Hough transform assumes that the middle of the road and the horizon line are roughly in the middle of the image. However if the car is going uphill or downhill that assumption is false and this could change our lane prediction. 
This problem could be solved by changing the shape of the polygon knowing the roll and pitch values of the car (using for instance some other sensors such as IMUs). 

All the images that we were given had only lane markings on the road. However one could imagine having other types of writings (such as railroad crossing) that could break the lane prediction by adding random lines in the area of interest. 

Also, it is to be noted that nothing has been done to reduce the processing time. All of the recognition in cars must be performed on the go, so taking understanding how to improve the time it takes to execute the lane recognition could be another path for improvement. An idea in this area would be to compute the Canny edge detector only on a limited area 

###3. Possible improvements to thepipeline

Possible improvements:
1. Use linear regression to classify which points belong to which lane (right or left). This would remove the dependency on a fixed threshold value and would be extremely helpful if the road is turning. 
2. Improve the polygon for the area of interest when no lines are detected
3. Use the previous knowledge (line equation) to accept only values within a certain range for the next frame. There are very few cases in which the slope for the line representing one of the lanes would abruptly change, so we could improve our detection (and make it smoother) by using information from the previous frames.

###4. Challenge

I quickly tried to look at the challenge video but I didn't have the time to work a solution.
i noticed that the video was showing part of the car, so i would have modified the polygon for the area of interest in order to remove that part of the video. 
Another thing that I noticed that the pipeline was getting confused with the shadow from the trees. To solve this issue, I would have used the lane found in the previous frame to infer the new lane. 

