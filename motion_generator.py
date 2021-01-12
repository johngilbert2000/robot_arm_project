import numpy as np
import re
import copy

class MotionCommand:
	'''
	command is a tuple
	there are 6 types of command: center, away, move, wait, grab, release, move_immutable
	for example
	(typename, param)
	('center', {})
	('away', {})
	('move', {'X':350,'Y':350,'Z':350,'Rx':180,'Ry':135,'Rz':100})
	('grab', {})
	('sleep', {'t':t}) t in seconds
	('release', {})
	('move_immutable', {'X':350,'Y':350,'Z':350,'Rx':180,'Ry':135,'Rz':100})
	'''
	def __init__(self, typename, param=None):
		if param is None:
			param = {}
		# perform type check
		assert type(typename) == str and type(param) == dict
		allowed_typename = ['center','away','move','wait','grab','release','move_immutable']
		assert typename in allowed_typename
		if typename in ['move','move_immutable']:
			allowed_keys = ['X','Y','Z','Rx','Ry','Rz']
			for key in param:
				assert key in allowed_keys
			param_float = {}
			for key in param:
				tmptype = type(param[key])
				assert tmptype == int or tmptype == float
				param_float[key] = float(param[key]) # int to float
			self.param = copy.deepcopy(param_float)
		else:
			self.param = copy.deepcopy(param)
		self.typename =  copy.deepcopy(typename)

	def __str__(self):
		all_keys = ['X','Y','Z','Rx','Ry','Rz']
		ret = ''
		ret += self.typename
		ret += ','
		param_str = []
		for key in all_keys:
			if key in self.param:
				param_str.append("'%s':%f"%(key,self.param[key]))
		param_str = '{'+','.join(param_str)+'}'
		ret += param_str
		return ret

	def translate(self, dx, dy):
		# only "move" can translate
		if self.typename == "move":
			assert 'X' in self.param
			assert 'Y' in self.param
			self.param['X'] += dx
			self.param['Y'] += dy
		return self

	def rotate(self, rot_center_X, rot_center_Y, rot_deg):
		# only "move" can rotate
		if self.typename == "move":
			assert 'X' in self.param
			assert 'Y' in self.param
			assert 'Rz' in self.param
			tmpX = self.param['X'] - rot_center_X
			tmpY = self.param['Y'] - rot_center_Y
			Sr = np.sin(np.deg2rad(rot_deg))
			Cr = np.cos(np.deg2rad(rot_deg))
			newX = Cr*tmpX - Sr*tmpY + rot_center_X
			newY = Sr*tmpX + Cr*tmpY + rot_center_Y
			newRz = self.param['Rz'] + rot_deg
			self.param['Rz'] = newRz
			self.param['X'] = newX
			self.param['Y'] = newY
		return self

class Motion:
	'''
	motion is a list of commands
	in which 'X', 'Y', 'Z', 'Rx', 'Ry', 'Rz' exist in param
	for all "move" commands
	'''
	def __init__(self, _MCList):
		assert type(_MCList) == list
		all_keys = ['X','Y','Z','Rx','Ry','Rz']
		currPose = {}
		defaultPoseSet = False
		tmplist = []
		for MC in _MCList:
			if MC.typename in ['move','move_immutable']:
				# the first "move" command must contain all 6 parameters
				if defaultPoseSet:
					for key in MC.param:
						currPose[key] = MC.param[key]
				else:
					for key in all_keys:
						assert key in MC.param
						currPose[key] = MC.param[key]
					defaultPoseSet = True
				newMC = MotionCommand(MC.typename, currPose)
			else:
				newMC = MC
			tmplist.append(newMC)
		self.MCList = copy.deepcopy(tmplist)

	def __str__(self):
		ret = ''
		for i in range(len(self.MCList)):
			ret += '%s: %s\n'% (i+1,self.MCList[i])
		return ret

	def translate(self, dx, dy):
		for i in range(len(self.MCList)):
			self.MCList[i].translate(dx, dy)
		return self

	def rotate(self, rot_center_X, rot_center_Y, rot_deg):
		for i in range(len(self.MCList)):
			self.MCList[i].rotate(rot_center_X, rot_center_Y, rot_deg)
		return self


def GetSpecialMotion(motion_name):
	# return a list of MotionCommand
	# not Motion
	ret = []
	if motion_name == 'take_gripper':
		take_gripper_safe = {'Z':400}
		take_gripper_rotation = {'Rx':135,'Ry':0}
		take_gripper_position = {'Z':320}
		ret.append(MotionCommand("move", take_gripper_safe))
		ret.append(MotionCommand("release", {}))
		ret.append(MotionCommand("move", take_gripper_rotation))
		ret.append(MotionCommand("move", take_gripper_position))
		ret.append(MotionCommand("grab", {}))
		ret.append(MotionCommand("move", take_gripper_safe))
		ret.append(MotionCommand("release", {}))
	if motion_name == 'put_gripper':
		put_gripper_safe = {'Z':400}
		put_gripper_rotation = {'Rx':135,'Ry':0}
		put_gripper_position = {'Z':320}
		ret.append(MotionCommand("move", put_gripper_safe))
		ret.append(MotionCommand("grab", {}))
		ret.append(MotionCommand("move", put_gripper_rotation))
		ret.append(MotionCommand("move", put_gripper_position))
		ret.append(MotionCommand("release", {}))
		ret.append(MotionCommand("move", put_gripper_safe))
	else:
		pass
	return ret

def PrintRawMCList(MCList):
	for i in range(len(MCList)):
		print('%s: %s'% (i+1,MCList[i]))
	return

def DemoTakeSomethingWithGripper():
	start = [MotionCommand("move_immutable",{'X':340,'Y':340,'Z':400,'Rx':180,'Ry':0,'Rz':135})]
	end = [MotionCommand("away")]

	gripper_location = MotionCommand("move",{'X':400,'Y':200,'Rz':160})
	take_gripper = [gripper_location]+GetSpecialMotion("take_gripper")
	put_gripper = [gripper_location]+GetSpecialMotion("put_gripper")

	item_location = MotionCommand("move",{'X':150,'Y':400,'Rz':100})
	grab = [MotionCommand("move",{'Z':220}),MotionCommand("grab",{}),MotionCommand("move",{'Z':400})]
	new_item_location = MotionCommand("move",{'X':180,'Y':370,'Rz':100})
	release = [MotionCommand("move",{'Z':220}),MotionCommand("release",{}),MotionCommand("move",{'Z':400})]
	item_subroutine = [item_location]+grab+[new_item_location]+release

	MCList = \
		start + \
	 	take_gripper + \
	 	item_subroutine + \
	 	put_gripper + \
	 	end

	print('Raw Motion Commands:')
	PrintRawMCList(MCList)

	print('\n\n=======================\n\n')

	m = Motion(MCList)
	print('Decoded Motion:')
	print(m)
	return

if __name__ == "__main__":
	DemoTakeSomethingWithGripper()
