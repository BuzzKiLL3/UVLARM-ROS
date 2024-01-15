


obstacles= []
angle= scanMsg.angle_min
for aDistance in scanMsg.ranges :
    if 0.1 < aDistance and aDistance < 5.0 :
        aPoint= [   
            math.cos(angle) * aDistance,
            math.sin(angle) * aDistance
        ]
        obstacles.append(aPoint)
    angle+= scanMsg.angle_increment

sample= [ [ round(p[0], 2), round(p[1], 2) ] for p in  obstacles[10:20] ]
self.get_logger().info( f" obs({len(obstacles)}) ...{sample}..." )

aPoint= Point32()
aPoint.x= (float)(math.cos(angle) * aDistance)
aPoint.y= (float)(math.sin( angle ) * aDistance)
aPoint.z= (float)(0)
