Braitenberg vehicle
===================

Note : 
--------
The more the range of the sonar, faster the wheel moves.., So, the range factor in actual braitenberg vehicles and in this configuration changes. because in actual scenario the more closer to source the more value you get,, but in our simulator, the far you are, the range of sonar is more. so the behaviours shown by A and B in our simulator is opposite to the actual Braitenberg vehicle behaviour. Explained below.


Type A: Aggressive Behaviour
-----------------------------
Speed : left wheel = (range from left sonar)*factor1_;
		right wheel = (range from right sonar)*factor1_;
when vehicle is closer to wall on left, the range in left sonar is less comparative to right sonar range, so it crashes and vice versa when vehicle near to right wall. This kind of behaviour is known as aggresive behaviour(opposite to braitenberg vehicle Type A)

Type B: Cowardly behaviour
---------------------------
Speed : left wheel = (range from right sonar)*factor1_;
		right wheel = (range from left sonar)*factor1_;
when vehicle is appraching a wall, according to range received by sonars, if vehicle is closer to left wall, the range of right sonar is more and so the left wheel rotates faster which saves it from crashing and vice versa when vehicle appraches right wall. This kind of behaviour is known as cowardly behaviour.

Type C: Mixed Behaviour
------------------------
Speed : left wheel = (range from right sonar)*factor1_+ (range from left sonar)*factor2_;
		right wheel = (range from left sonar)*factor1_ + (range from right sonar)*factor2_;
In this case as input to both the wheels is from both the sonars, the input to both the wheels is equal from sinars but it varries according to the factor 1 and factor 2 value defined for the Vehicle Type. So if the Factor_1 > Factor_2, the vehicle behaves like Type B(Cowardly), and when factor_2 > factor_1, the vehicle bahaves like Type A(Aggressive) as per our simulator. We call this behaviour as Mixed behaviour.