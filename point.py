"""
should include z for completeness
"""

class Point:
	def __init__(self,x,y,yaw):
		self.x=x
		self.y=y
		self.yaw=yaw

	def __sub__(self,other):
		return Point(self.x-other.x,self.y-other.y,self.yaw-other.yaw)

	def __str__(self):
		output="x: "+str(self.x)+" , "+"y: "+str(self.y)+" , "+"yaw: "+str(self.yaw)
		return output
	
	def __eq__(self,other):
		if self.x == other.x and self.y == other.y and self.yaw == other.yaw:
			return True
		else:
			return False

	# def __abs__(self):
	# 	return Point(abs(self.x),abs(self.y),abs(self.yaw))