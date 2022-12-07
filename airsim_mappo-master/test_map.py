import  airsim


a = airsim.client.MultirotorClient("192.168.0.100")
a.confirmConnection()
# a.moveByVelocityAsync()
# Z = 5.0
# sleepRate = 30
# swarm = Crazyswarm()
# timeHelper = swarm.timeHelper
# allcfs = swarm.allcfs
# cf = swarm.allcfs.crazyflies[0]

# allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
# a.armDisarm(True)
# a.enableApiControl(True)
center = airsim.Vector3r(0, 0, -3)
a.simCreateVoxelGrid(center, 60, 50, 12, 0.2, "111_222_333")