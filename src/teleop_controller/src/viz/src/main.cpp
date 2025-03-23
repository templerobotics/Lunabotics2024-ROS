#include <GL/gl3w.h>         // Ensure this is included before GLFW
#include <GLFW/glfw3.h>
#include <iostream>

#include "gui.h"

int main() {
    // Initialize GLFW
	if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }

    // Set OpenGL version (adjust as needed)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    // Create a window
    GLFWwindow* window = glfwCreateWindow(1000, 800, "Viz", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize gl3w
    if (gl3wInit()) {
        std::cerr << "Failed to initialize OpenGL using gl3w\n";
        return -1;
    }

    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << "\n";

    // GUI initialization
    setupGUI(window);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        renderGUI();

        glfwSwapBuffers(window);
    }

    cleanupGUI();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
