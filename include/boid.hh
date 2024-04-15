#pragma once
#include <vector>
#include <GLFW/glfw3.h>
#include <linmath.h>

#define INITIAL_BIAS_VALUE 0.000000004f
#define MAX_BIAS_VALUE 0.00001f
#define BIAS_INCREMENT 0.00000001f
#define EDGE_MARGIN 0.05f
#define EDGE_FORCE 0.0003f
#define MAX_SPEED 0.003f
#define MIN_SPEED 0.001f

typedef enum {
    GROUP_A,
    GROUP_B
} group_t;

class Boid {
public:
    Boid(unsigned int _index, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce);
    Boid(unsigned int _index, vec2 pos, GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce);
    virtual ~Boid();
    void update(const std::vector<Boid> &boids);
    void getPosition(vec2& pos);
    void getColour(vec3& col);
    unsigned int getIndex() { return index; }
    void setGroup(group_t _group);
    void setParameters(GLfloat _visualRange, GLfloat _protectedRange, GLfloat _separationForce, GLfloat _alignmentForce, GLfloat _cohesionForce);
private:
    void edgeAvoidance();
    void bias();
    void updateBias();
    unsigned int index;
    vec2 position;
    vec2 velocity;
    vec3 colour;
    GLfloat visualRange;
    GLfloat visualRangeSquared;
    GLfloat protectedRange;
    GLfloat protectedRangeSquared;
    GLfloat separationForce;
    GLfloat alignmentForce;
    GLfloat cohesionForce;
    GLfloat biasValue;
    group_t group = GROUP_A;
};