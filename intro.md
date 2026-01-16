
The core of the technique is as follows, given a camera and its position:
1. First, we take the newest frame of video
2. For every pixel, we subtract the value of that pixel from the previous frame, leaving us with a difference mask
3. Now, since we know the camera's position, and the size of the frustum, we calculate a ray for every pixel.
4. We now project each ray into a voxel grid, and for every voxel that a specific ray hits, adding the value of the origin pixel's difference.
If we repeat the above steps for multiple cameras in different locations, and raymarch into a shared voxel grid, we are able to detect moving objects with a very high degree of accuracy. This is due to the fact that the cost of our system increases linearly with every camera added, but the amount of information gained grows exponentially.

There are a few main things we need to work on, and some technical decisions we need to make surrounding them. Here's a short list, I will expand upon it shortly: we need to write the camera code, the main aggregating server, a client for the server that displays our data, and we need a simulator that can act as fake cameras, so that we can test the entire implementation.
1. The Camera
	1. We need to decide whether we are raymarching on the camera unit or the central aggregating server
	2. We need to decide what networking protocols to use.
	3. Normal camera vs event camera
	4. Fish-eye lens for 360 view?
	5. Maybe the most important question: how do we know where the camera is? GPS? Astrometric plate solving? Magnetometer?
	6. Frustum casting / cone casting?
	7. Do we attenuate based on distance?
2. The Main Server
	1. How do we filter our voxels? Do we just take the highest 99% / frame?
	2. How do we synchronize the data between our cameras?
	3. How do we deal with latency?
	4. What's the best data structure for our voxel grid? Do we use an octree that splits when it gets hit by a ray? Naive voxel grid has explosive memory requirements and not enough resolution.
3. The Client
	1. Web-based, we should pull satellite imagery to make the map pretty. We should be able to see points of identified objects, click on them, see which cameras are tracking them. The control schema should be very grand-strategy game.
4. The Simulator
	1. Bevy based. Should be pretty easy to set up. Just create and position cameras, then use our camera code. We can build this up over time, this is just for easy verification.
