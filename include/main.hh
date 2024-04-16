#include <ImFrame.h>
#include <vector>
#include <time.h>
#include <stdarg.h>
#include <stdio.h>
#include <iostream>
#include <assert.h>
#include "boid.hh"

#define GL_LOG_FILE "gl.log"
#define MAX_BOIDS 500
#define INITIAL_BOID_COUNT 100

#ifdef IMFRAME_WINDOWS
#include <SDKDDKVer.h>
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#endif

typedef struct {
    vec2 pos;
    vec3 col;
} Point;

class MainApp : public ImFrame::ImApp
{
public:
    MainApp(GLFWwindow * window) : ImFrame::ImApp(window) {
        Init();
        m_showParams = ImFrame::GetConfigValue("show", "params", m_showParams);
	    m_showAbout = ImFrame::GetConfigValue("show", "about", m_showAbout);
    }
    virtual ~MainApp() {
        glDeleteProgram(program);
        glDeleteShader(vertex_shader);
        glDeleteBuffers(1, &vertex_array);

        ImFrame::SetConfigValue("show", "params", m_showParams);
        ImFrame::SetConfigValue("show", "about", m_showAbout);
    }
    void OnUpdate() override;
    void OnKeyPress(int key, int mods) override;
private:
    const char * vertex_shader_text =
        R"r(
        #version 330
        uniform mat4 MVP;
        in vec3 vCol;
        in vec2 vPos;
        out vec3 colour;
        void main()
        {
            gl_Position = MVP * vec4(vPos, 0.0, 1.0);
            colour = vCol;
        }
        )r";

    const char * fragment_shader_text =
        R"r(
        #version 330
        in vec3 colour;
        out vec4 fragment;
        void main()
        {
            fragment = vec4(colour, 1.0);
        }
        )r";

    GLuint vertex_shader;
    GLint program;
    GLint mvp_location;
    GLint vpos_location;
    GLint vcol_location;
    GLuint vertex_array;

    std::vector<Point> points;
    std::vector<Boid> boids;

    GLfloat visualRange = 0.1f;
    GLfloat protectedRange = 0.04f;
    GLfloat separationForce = 0.004f;
    GLfloat alignmentForce = 0.03f;
    GLfloat cohesionForce = 0.00025f;
    GLfloat biasValue = 0.0000007f;

    GLfloat minSpeed = 0.001f;
    GLfloat maxSpeed = 0.003f;

    unsigned int count = INITIAL_BOID_COUNT;

    bool m_showParams = false;
    bool m_showAbout = false;

    static bool restart_gl_log();
    static bool gl_log(const char* message, ...);
    static bool gl_log_err(const char* message, ...);
    static void glfw_error_callback(int error, const char* description);
    static void log_gl_params();
    void _update_fps_counter(GLFWwindow* window);
    void Init();
    void UpdateBoids();
    void ShowBoids();
    void updateBoidCount();
    void ShowParamEditor(bool * p_open = nullptr);
};