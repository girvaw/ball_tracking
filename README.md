# ball_tracking

## Tracks a ball and publishes the position

Subscribes to the following topics:
- kinect2/sd/image_color_rect
- kinect2/sd/image_depth
- tracking_params

## Filters the images then identifies circles in the filtered image.

Publishes to the following topics:
- five_axis_image
- five_axis_depth_image
- ball_target_point
