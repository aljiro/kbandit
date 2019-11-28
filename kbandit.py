#!/usr/bin/python

import numpy as np
import time

class MiroController:

	def __init__( self ):
		self.actions = [ self.earWiggle, self.tailWag, self.rotate, self.nod ]

	# Actions
	def earWiggle( self ):
		print "wiggle"

	def tailWag( self ):
		
		print "wag"

	def rotate( self ):
		print "rotate"

	def nod( self ):
		print "nod"

	# Listeners
	def touchListener( self ):
		pass

	# Main loop
	def run( self ):
		running = True
		count = 0

		while( running ):
			if count <= 0:
				# Action selection
				r = np.random.randint( 0, len(self.actions) )
				# Action execution
				self.actions[r]()
				count = 100
			else:
				count = count - 1

			time.sleep(0.01)

if __name__ == "__main__":
	mc = MiroController()
	mc.run()