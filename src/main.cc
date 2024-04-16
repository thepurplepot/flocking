#include "main.hh"

bool MainApp::restart_gl_log() {
    FILE* file = fopen(GL_LOG_FILE, "w");
    if (!file) {
        fprintf(stderr,
        "ERROR: could not open GL_LOG_FILE log file %s for writing\n",
        GL_LOG_FILE);
        return false;
    }
    time_t now = time(NULL);
    char* date = ctime(&now);
    fprintf(file, "GL_LOG_FILE log. local time %s\n", date);
    fclose(file);
    return true;
}

bool MainApp::gl_log(const char* message, ...) {
    va_list argptr;
    FILE* file = fopen(GL_LOG_FILE, "a");
    if (!file) {
        fprintf(
        stderr,
        "ERROR: could not open GL_LOG_FILE %s file for appending\n",
        GL_LOG_FILE
        );
        return false;
    }
    va_start(argptr, message);
    vfprintf(file, message, argptr);
    va_end(argptr);
    fclose(file);
    return true;
}

void MainApp::glfw_error_callback(int error, const char* description) {
    gl_log_err("GLFW ERROR: code %i msg: %s\n", error, description);
}

bool MainApp::gl_log_err(const char* message, ...) {
    va_list argptr;
    FILE* file = fopen(GL_LOG_FILE, "a");
    if (!file) {
        fprintf(stderr,
        "ERROR: could not open GL_LOG_FILE %s file for appending\n",
        GL_LOG_FILE);
        return false;
    }
    va_start(argptr, message);
    vfprintf(file, message, argptr);
    va_end(argptr);
    va_start(argptr, message);
    vfprintf(stderr, message, argptr);
    va_end(argptr);
    fclose(file);
    return true;
}

void MainApp::log_gl_params() {
    GLenum params[] = {
        GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS,
        GL_MAX_CUBE_MAP_TEXTURE_SIZE,
        GL_MAX_DRAW_BUFFERS,
        GL_MAX_FRAGMENT_UNIFORM_COMPONENTS,
        GL_MAX_TEXTURE_IMAGE_UNITS,
        GL_MAX_TEXTURE_SIZE,
        GL_MAX_VARYING_FLOATS,
        GL_MAX_VERTEX_ATTRIBS,
        GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS,
        GL_MAX_VERTEX_UNIFORM_COMPONENTS,
        GL_MAX_VIEWPORT_DIMS,
        GL_STEREO,
    };
    const char* names[] = {
        "GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS",
        "GL_MAX_CUBE_MAP_TEXTURE_SIZE",
        "GL_MAX_DRAW_BUFFERS",
        "GL_MAX_FRAGMENT_UNIFORM_COMPONENTS",
        "GL_MAX_TEXTURE_IMAGE_UNITS",
        "GL_MAX_TEXTURE_SIZE",
        "GL_MAX_VARYING_FLOATS",
        "GL_MAX_VERTEX_ATTRIBS",
        "GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS",
        "GL_MAX_VERTEX_UNIFORM_COMPONENTS",
        "GL_MAX_VIEWPORT_DIMS",
        "GL_STEREO",
    };
    gl_log("GL Context Params:\n");
    // integers - only works if the order is 0-10 integer return types
    for (int i = 0; i < 10; i++) {
        int v = 0;
        glGetIntegerv(params[i], &v);
        gl_log("%s %i\n", names[i], v);
    }
    // others
    int v[2];
    v[0] = v[1] = 0;
    glGetIntegerv(params[10], v);
    gl_log("%s %i %i\n", names[10], v[0], v[1]);
    unsigned char s = 0;
    glGetBooleanv(params[11], &s);
    gl_log("%s %u\n", names[11], (unsigned int)s);
    gl_log("-----------------------------\n");
}

void MainApp::_update_fps_counter(GLFWwindow* window) {
    static double previous_seconds = glfwGetTime();
    static int frame_count;
    double current_seconds = glfwGetTime();
    double elapsed_seconds = current_seconds - previous_seconds;
    if (elapsed_seconds > 0.25) {
        previous_seconds = current_seconds;
        double fps = (double)frame_count / elapsed_seconds;
        char tmp[128];
        snprintf(tmp, 128, "Flocking opengl @ fps: %.2f", fps);
        glfwSetWindowTitle(window, tmp);
        frame_count = 0;
    }
    frame_count++;
}

void MainApp::OnUpdate() {
    if (ImFrame::BeginMainMenuBar())
	{
		if (ImFrame::BeginMenu("View"))
		{
            ImFrame::MenuItem("Show Params", nullptr, &m_showParams);
            ImFrame::EndMenu();
		}
        if (ImFrame::BeginHelpMenu("Help", true))
        {
            if (ImFrame::MenuItem("About", nullptr))
                m_showAbout = true;
            ImFrame::EndMenu();
        } 
        ImFrame::EndMainMenuBar();
    }

    if (m_showParams) {
        ShowParamEditor();
    }
    if (m_showAbout) {
        ImGui::OpenPopup("About Flocking");
        // Always center this window when appearing
        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
        if (ImGui::BeginPopupModal("About Flocking", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
        {
            ImGui::Text("Flocking implements the boids flocking algorithm");
            if (ImGui::Button("Close"))
            {
                ImGui::CloseCurrentPopup();
                m_showAbout = false;
            }
            ImGui::EndPopup();
        }
    }

    UpdateBoids();
	ShowBoids();
}

void MainApp::ShowParamEditor(bool * p_open)
{
    ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Parameters", p_open))
    {
        ImGui::End();
        return;
    }

    ImGui::Text("Visual Range");
    ImGui::SliderFloat("##VisualRange", &visualRange, 0.0f, 0.5f);
    ImGui::Text("Protected Range");
    ImGui::SliderFloat("##ProtectedRange", &protectedRange, 0.0f, 0.5f);
    ImGui::Text("Separation Force");
    ImGui::SliderFloat("##SeparationForce", &separationForce, 0.0f, 0.1f, "%.4f");
    ImGui::Text("Alignment Force");
    ImGui::SliderFloat("##AlignmentForce", &alignmentForce, 0.0f, 0.1f, "%.4f");
    ImGui::Text("Cohesion Force");
    ImGui::SliderFloat("##CohesionForce", &cohesionForce, 0.0f, 0.01f, "%.5f");
    ImGui::Text("Bias Velocity");
    ImGui::SliderFloat("##BiasVelocity", &biasValue, 0.0f, 0.0001f, "%.8f");
    if (ImGui::Button("Update Parameters")) {
        for (size_t i = 0; i < boids.size(); i++) {
            boids[i].setParameters(visualRange, protectedRange, separationForce, alignmentForce, cohesionForce, biasValue);
        }
    }
    ImGui::Separator();
    ImGui::Text("Speed Limits");
    ImGui::SliderFloat("##MinSpeed", &minSpeed, 0.0f, 0.01f, "%.5f");
    ImGui::SliderFloat("##MaxSpeed", &maxSpeed, 0.0f, 0.01f, "%.5f");
    if (ImGui::Button("Update Speed Limits")) {
        for (size_t i = 0; i < boids.size(); i++) {
            boids[i].setSpeedLimits(minSpeed, maxSpeed);
        }
    }
    ImGui::Separator();
    ImGui::Text("Boid Count");
    ImGui::SliderInt("##BoidCount", (int*)&count, 0, MAX_BOIDS);
    if (ImGui::Button("Update Boid Count")) {
        updateBoidCount();
    }
    ImGui::Separator();
    if (ImGui::Button("Randomise Positions")) {
        for (size_t i = 0; i < boids.size(); i++) {
            boids[i].randomisePosition();
        }
    }
    if (ImGui::Button("Reset")) {
        visualRange = 0.1f;
        protectedRange = 0.04f;
        separationForce = 0.004f;
        alignmentForce = 0.03f;
        cohesionForce = 0.00025f;
        count = MAX_BOIDS;
        biasValue = 0.000000004f;
        for (size_t i = 0; i < boids.size(); i++) {
            boids[i].setParameters(visualRange, protectedRange, separationForce, alignmentForce, cohesionForce, biasValue);
        }
        updateBoidCount();
    }

    ImGui::End();
}

void MainApp::OnKeyPress(int key, int mods)
{
	if (key == GLFW_KEY_ESCAPE)
		glfwSetWindowShouldClose(GetWindow(), GLFW_TRUE);
    if (mods == GLFW_MOD_CONTROL && key == GLFW_KEY_P) {
        m_showParams = !m_showParams;
    }
}

void MainApp::Init()
{
    for (size_t i = 0; i < INITIAL_BOID_COUNT; i++) {
        Boid boid(i, visualRange, protectedRange, separationForce, alignmentForce, cohesionForce, biasValue);
        if (i % 2 == 0) {
            boid.setGroup(GROUP_A);
        } else {
            boid.setGroup(GROUP_B);
        }
        boids.push_back(boid);
    }

    assert(restart_gl_log());
    gl_log("starting GLFW\n%s\n", glfwGetVersionString());
    glfwSetErrorCallback(MainApp::glfw_error_callback);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    log_gl_params();

    // NOTE: OpenGL error checks have been omitted for brevity

	GLuint vertex_buffer;
	glGenBuffers(1, &vertex_buffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Point)*MAX_BOIDS, points.data(), GL_STATIC_DRAW);

	vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
	glCompileShader(vertex_shader);

	const GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
	glCompileShader(fragment_shader);

	program = glCreateProgram();
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);

	mvp_location = glGetUniformLocation(program, "MVP");
	vpos_location = glGetAttribLocation(program, "vPos");
	vcol_location = glGetAttribLocation(program, "vCol");

	glGenVertexArrays(1, &vertex_array);
	glBindVertexArray(vertex_array);
	glEnableVertexAttribArray(vpos_location);
	glVertexAttribPointer(vpos_location, 2, GL_FLOAT, GL_FALSE,
		sizeof(Point), (void *)offsetof(Point, pos));
	glEnableVertexAttribArray(vcol_location);
	glVertexAttribPointer(vcol_location, 3, GL_FLOAT, GL_FALSE,
		sizeof(Point), (void *)offsetof(Point, col));
}

void MainApp::UpdateBoids()
{
    points.clear();
    for (size_t i = 0; i < boids.size(); i++) {
        Boid &boid = boids[i];
        boid.update(boids);
        vec2 pos;
        boid.getPosition(pos);
        vec3 col;
        boid.getColour(col);
        Point p = { {pos[0], pos[1]}, {col[0], col[1], col[2]} };
        points.push_back(p);
    }
}

void MainApp::updateBoidCount()
{
    if (count == boids.size() || count == 0) {
        return;
    }
    if (count > MAX_BOIDS) {
        count = MAX_BOIDS;
    }
    if (count > boids.size() ) {
        for (size_t i = boids.size(); i < count; i++) {
            Boid boid(i, visualRange, protectedRange, separationForce, alignmentForce, cohesionForce, biasValue);
            if (i % 2 == 0) {
                boid.setGroup(GROUP_A);
            } else {
                boid.setGroup(GROUP_B);
            }
            boids.push_back(boid);
        }
    } else {
        size_t size = boids.size();
        for (size_t i = count; i < size; i++) {
            boids.pop_back();
            boids.shrink_to_fit();       
        }
    }
}

void MainApp::ShowBoids()
{
    int width, height;
    auto window = GetWindow();
	glfwGetFramebufferSize(window, &width, &height);
	const float ratio = width / (float)height;

    _update_fps_counter(window);

    glClearColor(0.6f, 0.6f, 0.8f, 1.0f);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Point)*points.size(), points.data(), GL_STATIC_DRAW);

	mat4x4 m, p, mvp;
	mat4x4_identity(m);
	//mat4x4_rotate_Z(m, m, (float)glfwGetTime());
	mat4x4_ortho(p, -ratio, ratio, -1.f, 1.f, 1.f, -1.f);
	mat4x4_mul(mvp, p, m);

	glUseProgram(program);
	glUniformMatrix4fv(mvp_location, 1, GL_FALSE, (const GLfloat *)&mvp);
	glBindVertexArray(vertex_array);
    glPointSize(7.0f);
	glDrawArrays(GL_POINTS, 0, points.size());
}

// ImFrame main function and app creation
IMFRAME_MAIN("ImFrame", "Flocking", MainApp)

