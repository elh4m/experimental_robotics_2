(define (problem task)
(:domain turtlebot)
(:objects
    wp0 wp1 wp2 wp3 wp4 - waypoint
    kenny - robot
)
(:init
    (= (charge kenny) 0)
    (robot_at kenny wp0)
    (docked kenny)
    (dock_at wp0)
    (= (hint_collected kenny) 0)
)
(:goal (and
    (visited wp0)
    (visited wp1)
    (visited wp2)
    (visited wp3)
    (visited wp4)
    (docked kenny)
    (> (charge kenny) 0)
)))
