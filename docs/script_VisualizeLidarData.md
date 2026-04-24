**🚀 LiDAR Visualization Workflow (with terminals)**

**🧠 Key idea**

You need **multiple terminals running at the same time**:

- one → playing the data
- one → checking topics (optional but useful)
- one → visualization (RViz)

**🖥️ Terminal 1 → Play the rosbag (DATA SOURCE)**

Open first terminal:

source /opt/ros/humble/setup.bash  
ros2 bag play /workspace/studentProject --loop

👉 This is **mandatory**  
👉 This terminal must stay running  
👉 It is the one that “generates” the LiDAR data

**🖥️ Terminal 2 → Verify data (DEBUG / CHECK)**

Open a **second terminal**:

source /opt/ros/humble/setup.bash  
ros2 topic list

You should see:

/velodyne/points_raw  
/PointCloudDetection

Then check if data is flowing:

ros2 topic hz /velodyne/points_raw

👉 If you see a frequency → everything is working  
👉 If not → bag is not playing

**🖥️ Terminal 3 → Visualization (RViz)**

Open a **third terminal**:

source /opt/ros/humble/setup.bash  
rviz2

**🎯 Inside RViz**

**1\. Set Fixed Frame**

Top-left → Global Options

Try:

velodyne

If it fails:

- velodyne_link
- base_link

**2\. Add LiDAR point cloud**

- Click **Add**
- Select **PointCloud2**

Then set:

Topic → /velodyne/points_raw

**3\. Adjust visualization**

- Style → Points
- Size (Pixels) → 2 or 3
- Color Transformer → Intensity

**🧪 Optional (extra useful)**

**Add processed cloud**

Add another PointCloud2:

/PointCloudDetection

**Add camera**

Add Image display:

/blackfly_s/cam0/image_rectified

**⚠️ Important rules**

**❗ Rule 1: Terminal 1 must always run**

If you stop:

ros2 bag play ...

👉 RViz will show nothing

**❗ Rule 2: RViz depends on topics**

No topics → no visualization

**❗ Rule 3: Order matters**

Correct order:

1\. Start bag (Terminal 1)  
2\. Check topics (Terminal 2)  
3\. Open RViz (Terminal 3)

**🔥 Mental model**

Think of it like this:

Terminal 1 → produces data  
Terminal 2 → inspects data  
Terminal 3 → visualizes data
