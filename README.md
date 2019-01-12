# Camera-distance-estimation-using-LiDAR-tool

This program finds a matrix that matches camera pixel values with the distance values of LiDAR.  
And this works with pyqt and ros.    
  
You just need to match ros message type.  
- LiDAR message type : sensor_msgs - PointCloud2  
- Camera message type : sensor_msgs - Image    

Here is the working video link: https://www.youtube.com/watch?v=ovC2AM9LdPY  

![screenshot from 2019-01-13 05-10-12](https://user-images.githubusercontent.com/25835750/51078345-0948a600-16f7-11e9-866b-945931be2bc9.png)  

Match the four points pt1, pt2, pt3, p4 of the image on the right with the four points pt1', pt2', pt3', pt4' corresponding to the distance of the left LiDAR data
