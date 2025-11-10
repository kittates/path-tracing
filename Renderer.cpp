//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include <thread>
#include "Renderer.hpp"


inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

const float EPSILON = 0.00001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    // change the spp value to change sample ammount
    // spp设置每个像素采样的次数
    int spp = 4;
    // int spp = 512;
    std::cout << "SPP: " << spp << "\n";

    //-------注释------
    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    //-----注释------
    // 多线程优化
    // ATTENTION!! do not change the num_threads value
    int num_threads = 4;    
    std::vector<std::thread> threads(num_threads);
    std::mutex mtx; // 用于同步输出进度
    int thread_height = scene.height / num_threads; // 每个线程负责的行数
    float process=0;    // 全局渲染速度
    float Reciprocal_Scene_height=1.f/ (float)scene.height; // 每行对应的进度增量

    auto renderRows = [&](int thread_index) {
        int height = thread_height * (thread_index + 1);
        for(uint32_t j=height - thread_height; j<height; j++) { 
            for(uint32_t i=0; i<scene.width; i++) {
                
                for (int k = 0; k < spp; k++){
                    
                    // 在单个像素内进行随机扰动采样
                    // float x = (2 * (i + get_random_float()-0.5f + EPSILON) / (float)scene.width - 1) * imageAspectRatio * scale;
                    // float y = (1 - 2 * (j + get_random_float()-0.5f - EPSILON) / (float)scene.height) * scale;
                    
                    float x = (2 * (i + 0.5f) / (float)scene.width - 1) * imageAspectRatio * scale;
                    float y = (1 - 2 * (j + 0.5f) / (float)scene.height) * scale;

                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    // 这里不再使用m++
                    framebuffer[j * scene.width + i] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
                }
                // m++;
            }
            // 互斥更新进度条
            mtx.lock();
            process += Reciprocal_Scene_height;
            UpdateProgress(process);
            mtx.unlock();
        }
    };
    
    for (int k = 0; k < num_threads; k++)
    {
        threads[k] = std::thread(renderRows,k);
    }
    for (int k = 0; k < num_threads; k++)
    {
        threads[k].join();  // 等待所有线程完成
    }

    UpdateProgress(1.f);

    
    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
