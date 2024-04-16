#include "boid.hh"
#include <random>
#include <iostream>

Boid::Boid(unsigned int _index, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce, GLfloat _biasValue) {
    index = _index;
    position[0] = rand() % 100 / 100.0f - 0.5f;
    position[1] = rand() % 100 / 100.0f - 0.5f;
    velocity[0] = rand() % 100 / 100.0f - 0.5f;
    velocity[1] = rand() % 100 / 100.0f - 0.5f;
    int c = rand() % 3;
    colour[0] = c == 0 ? 1.0f : 0.0f;
    colour[1] = c == 1 ? 1.0f : 0.0f;
    colour[2] = c == 2 ? 1.0f : 0.0f;
    setParameters(_visualRange, _protectedRange, _separationForce, _alignmentForce, _cohesionForce, _biasValue);
    setSpeedLimits(0.001f, 0.003f);
    biasValue = INITIAL_BIAS_VALUE;
}

Boid::Boid(unsigned int _index, vec2 pos, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce, GLfloat _biasValue) {
    index = _index;
    position[0] = pos[0];
    position[1] = pos[1];
    velocity[0] = rand() % 100 / 100.0f - 0.5f;
    velocity[1] = rand() % 100 / 100.0f - 0.5f;
    int c = rand() % 3;
    colour[0] = c == 0 ? 1.0f : 0.0f;
    colour[1] = c == 1 ? 1.0f : 0.0f;
    colour[2] = c == 2 ? 1.0f : 0.0f;
    setParameters(_visualRange, _protectedRange, _separationForce, _alignmentForce, _cohesionForce, _biasValue);
    setSpeedLimits(0.001f, 0.003f);
    biasValue = INITIAL_BIAS_VALUE;
}

void Boid::setParameters(GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce, GLfloat _biasValue) {
    visualRange = _visualRange;
    visualRangeSquared = visualRange * visualRange;
    protectedRange = _protectedRange;
    protectedRangeSquared = protectedRange * protectedRange;
    separationForce = _separationForce;
    alignmentForce = _alignmentForce;
    cohesionForce = _cohesionForce;
    if (!incrementBias) {
        biasValue = _biasValue;
    }
}

void Boid::setSpeedLimits(GLfloat _minSpeed, GLfloat _maxSpeed) {
    minSpeed = _minSpeed;
    maxSpeed = _maxSpeed;
}

Boid::~Boid() {
    
}

void Boid::getPosition(vec2& pos) {
    pos[0] = position[0];
    pos[1] = position[1];
}

void Boid::randomisePosition() {
    position[0] = rand() % 100 / 100.0f - 0.5f;
    position[1] = rand() % 100 / 100.0f - 0.5f;
}

void Boid::getColour(vec3& col) {
    col[0] = colour[0];
    col[1] = colour[1];
    col[2] = colour[2];
}

void Boid::randomiseVelocity() {
    velocity[0] = rand() % 100 / 100.0f - 0.5f;
    velocity[1] = rand() % 100 / 100.0f - 0.5f;
}

void Boid::setGroup(group_t _group) {
    group = _group;
    // if (group == GROUP_A) {
    //     colour[0] = 1.0f;
    // } else {
    //     colour[2] = 1.0f;
    // }
}

void Boid::update(const std::vector<Boid> &boids) {
    vec2 positionAvg = {0.0f, 0.0f};
    vec2 velocityAvg = {0.0f, 0.0f};
    vec3 colourAvg = {0.0f, 0.0f, 0.0f};
    unsigned int count = 0;
    vec2 close_d = {0.0f, 0.0f};

    for (const auto& boid : boids) {
        if (boid.index == index) {
            continue;
        }

        vec2 d = {boid.position[0] - position[0], boid.position[1] - position[1]};

        if (abs(d[0]) > visualRange || abs(d[1]) > visualRange) {
            continue;
        }

        GLfloat squaredDistance = d[0] * d[0] + d[1] * d[1];

        if (squaredDistance < protectedRangeSquared) {
            close_d[0] += position[0] - boid.position[0];
            close_d[1] += position[1] - boid.position[1];
        } else if (squaredDistance < visualRangeSquared) {
            positionAvg[0] += boid.position[0];
            positionAvg[1] += boid.position[1];
            velocityAvg[0] += boid.velocity[0];
            velocityAvg[1] += boid.velocity[1];
            colourAvg[0] += boid.colour[0];
            colourAvg[1] += boid.colour[1];
            colourAvg[2] += boid.colour[2];
            count++;
        }
    }

    if (count > 0) {
        positionAvg[0] /= count;
        positionAvg[1] /= count;
        velocityAvg[0] /= count;
        velocityAvg[1] /= count;
        colourAvg[0] /= count;
        colourAvg[1] /= count;
        colourAvg[2] /= count;

        velocity[0] += (positionAvg[0] - position[0]) * cohesionForce + (velocityAvg[0] - velocity[0]) * alignmentForce;
        velocity[1] += (positionAvg[1] - position[1]) * cohesionForce + (velocityAvg[1] - velocity[1]) * alignmentForce;
        updateColour(colourAvg);
    }
    velocity[0] += close_d[0] * separationForce;
    velocity[1] += close_d[1] * separationForce;
    edgeAvoidance();
    if(incrementBias)
        updateBias();
    bias();

    GLfloat speed = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
    if (speed < minSpeed) {
        velocity[0] *= minSpeed / speed;
        velocity[1] *= minSpeed / speed;
    } else if (speed > maxSpeed) {
        velocity[0] *= maxSpeed / speed;
        velocity[1] *= maxSpeed / speed;
    }
    position[0] += velocity[0];
    position[1] += velocity[1];
}

void Boid::edgeAvoidance() {
    if (position[0] < -1.0f + EDGE_MARGIN) {
        velocity[0] += EDGE_FORCE;
    }
    else if (position[0] > 1.0f - EDGE_MARGIN) {
        velocity[0] -= EDGE_FORCE;
    }
    if (position[1] < -1.0f + EDGE_MARGIN) {
        velocity[1] += EDGE_FORCE;
    }
    else if (position[1] > 1.0f - EDGE_MARGIN) {
        velocity[1] -= EDGE_FORCE;
    }
}

void Boid::bias() {
    if(group == GROUP_A) {
        velocity[0] = (1-biasValue) * velocity[0] + biasValue;
    } else {
        velocity[0] = (1-biasValue) * velocity[0] - biasValue;
    }
}

void Boid::updateBias() {
    if(group == GROUP_A) {
        if (velocity[0] > 0.0f) {
            biasValue += BIAS_INCREMENT;
            if(biasValue > MAX_BIAS_VALUE) {
                biasValue = MAX_BIAS_VALUE;
            }
        } else {
            biasValue -= BIAS_INCREMENT;
            if(biasValue < BIAS_INCREMENT) {
                biasValue = BIAS_INCREMENT;
            }
        }
    } else {
        if (velocity[0] < 0.0f) {
            biasValue += BIAS_INCREMENT;
            if(biasValue > MAX_BIAS_VALUE) {
                biasValue = MAX_BIAS_VALUE;
            }
        } else {
            biasValue -= BIAS_INCREMENT;
            if(biasValue < BIAS_INCREMENT) {
                biasValue = BIAS_INCREMENT;
            }
        }
    }
}

// TODO only called when group changes!!
void Boid::updateColour(vec3 colourAvg) {
    GLfloat blending = 0.7f;
    for (int i = 0; i < 3; i++) {
        colour[i] = blending * abs(position[i]) + (1 - blending) * colourAvg[i];
    }
}