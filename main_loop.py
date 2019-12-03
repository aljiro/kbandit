#!/usr/bin/python

import numpy as np
import time

class MiroController:

	def __init__( self ):
		print "Initializing the controller"
		self.actions = [ self.earWiggle, self.tailWag, self.rotate, self.shine ]

	# Actions
	def earWiggle( self, t ):		
		print "wiggle"
		return False
		

	def tailWag( self, t ):
		print "wag"
		return False

		
	def rotate( self, t ):
		print "rotate"
		return False


	def shine( self, t ):		
		print "Shinnning"
		return False

	# Listeners
	def touchListener( self ):
		pass

	# Main loop
	def run( self ):
		running = True
		r = np.random.randint( 0, len(self.actions) )
		t = 0.0
		h = 0.1

		print "Starting the main loop"

		while( running ):
			if not self.actions[r]( t ):
				print "Action Finished, changing action"
				# Action selection
				r = np.random.randint( 0, len(self.actions) )
				t = 0.0
			
			t = t + h			
			time.sleep(0.05)

if __name__ == "__main__":
	mc = MiroController()
	mc.run()