# Flocking

<hr />

An ImFrame based cross-platform C++ app to simulate boid (flocking) behaviour. 
Based on implementation from: https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html

# Setup

Clone ImFrame with ImGUI and OpenGl dependencies using:
```bash
git submodule update --init --recursive
```

Build with:
```bash
cmake .
cmake --build .
```

Run with:
```bash
./bin/Flocking
```

### Todos

 - Implement different shapes for boids (triangles pointing in direction of velocity)
 - Allow user to change max/min velocity
 - Allow user to change boid groups/bias
 - More advanced boid features (object avoidance, limited FOV)
 - Optimisations ...

License
----

GPL-3.0
