#include "boid.hh"
#include <random>
#include <iostream>

Boid::Boid(unsigned int _index, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce) {
    index = _index;
    position[0] = rand() % 100 / 100.0f - 0.5f;
    position[1] = rand() % 100 / 100.0f - 0.5f;
    velocity[0] = rand() % 100 / 100.0f - 0.5f;
    velocity[1] = rand() % 100 / 100.0f - 0.5f;
    colour[0] = 1.0f;
    colour[1] = 1.0f;
    colour[2] = 1.0f;
    setParameters(_visualRange, _protectedRange, _separationForce, _alignmentForce, _cohesionForce);
    biasValue = INITIAL_BIAS_VALUE;
}

Boid::Boid(unsigned int _index, vec2 pos, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce) {
    index = _index;
    position[0] = pos[0];
    position[1] = pos[1];
    velocity[0] = rand() % 100 / 100.0f - 0.5f;
    velocity[1] = rand() % 100 / 100.0f - 0.5f;
    colour[0] = 1.0f;
    colour[1] = 1.0f;
    colour[2] = 1.0f;
    setParameters(_visualRange, _protectedRange, _separationForce, _alignmentForce, _cohesionForce);
    biasValue = INITIAL_BIAS_VALUE;
}

void Boid::setParameters(GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce) {
    visualRange = _visualRange;
    visualRangeSquared = visualRange * visualRange;
    protectedRange = _protectedRange;
    protectedRangeSquared = protectedRange * protectedRange;
    separationForce = _separationForce;
    alignmentForce = _alignmentForce;
    cohesionForce = _cohesionForce;
}

Boid::~Boid() {
    
}

void Boid::getPosition(vec2& pos) {
    pos[0] = position[0];
    pos[1] = position[1];
}

void Boid::getColour(vec3& col) {
    col[0] = colour[0];
    col[1] = colour[1];
    col[2] = colour[2];
}

void Boid::setGroup(group_t _group) {
    group = _group;
    if (group == GROUP_A) {
        colour[0] = 1.0f;
        colour[1] = 0.0f;
        colour[2] = 0.0f;
    } else {
        colour[0] = 0.0f;
        colour[1] = 0.0f;
        colour[2] = 1.0f;
    }
}

void Boid::update(const std::vector<Boid> &boids) {
    vec2 positionAvg = {0.0f, 0.0f};
    vec2 velocityAvg = {0.0f, 0.0f};
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
            count++;
        }
    }

    if (count > 0) {
        positionAvg[0] /= count;
        positionAvg[1] /= count;
        velocityAvg[0] /= count;
        velocityAvg[1] /= count;

        velocity[0] += (positionAvg[0] - position[0]) * cohesionForce + (velocityAvg[0] - velocity[0]) * alignmentForce;
        velocity[1] += (positionAvg[1] - position[1]) * cohesionForce + (velocityAvg[1] - velocity[1]) * alignmentForce;
    }
    velocity[0] += close_d[0] * separationForce;
    velocity[1] += close_d[1] * separationForce;
    edgeAvoidance();
    updateBias();
    bias();

    GLfloat speed = sqrt(velocity[0] * velocity[0] + velocity[1] * velocity[1]);
    if (speed < MIN_SPEED) {
        velocity[0] *= MIN_SPEED / speed;
        velocity[1] *= MIN_SPEED / speed;
    } else if (speed > MAX_SPEED) {
        velocity[0] *= MAX_SPEED / speed;
        velocity[1] *= MAX_SPEED / speed;
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