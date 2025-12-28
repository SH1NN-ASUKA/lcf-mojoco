// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/simple_car/simple_car.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>


namespace mjpc {

std::string SimpleCar::XmlPath() const {
  return GetModelPath("simple_car/task.xml");
}

std::string SimpleCar::Name() const { return "SimpleCar"; }

float speed = 0.0f;  // 当前车速（km/h）


// 绘制圆形边框
void SimpleCar::drawCircle(float radius, int segments) {
    glBegin(GL_LINE_LOOP);
    for (int i = 0; i < segments; i++) {
        float angle = 2 * M_PI * i / segments;
        glVertex2f(radius * cos(angle), radius * sin(angle));
    }
    glEnd();
}

void SimpleCar::drawTicks(float radius, int tickCount) {
    float angleStep = 180.0f / tickCount;  // 因为是半圆，刻度分布在180度内
    for (int i = 0; i < tickCount; i++) {
        float angle = i * angleStep * M_PI / 180.0f;
        float tickLength = 0.02f;
        float x1 = radius * cos(angle);
        float y1 = radius * sin(angle);
        float x2 = (radius - tickLength) * cos(angle);
        float y2 = (radius - tickLength) * sin(angle);
        
        glBegin(GL_LINES);
        glVertex2f(x1, y1);
        glVertex2f(x2, y2);
        glEnd();
    }
}

// 绘制速度指针
void SimpleCar::drawPointer(float angle) {
    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);  // 指针的根部
    glVertex2f(0.8f * cos(angle), 0.8f * sin(angle));  // 指针的尖端
    glEnd();
}

// 绘制数字
void SimpleCar::drawNumber(float radius, int number, float angle) {
    char buffer[10];
    snprintf(buffer, sizeof(buffer), "%d", number);
    glRasterPos2f(radius * cos(angle), radius * sin(angle));
    for (int i = 0; buffer[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, buffer[i]);
    }
}

// 绘制仪表盘
void SimpleCar::drawDashboard(float* dashboard_pos, float speed_ratio) {
   glClear(GL_COLOR_BUFFER_BIT);

    // 将仪表盘移动到正确的位置
    glPushMatrix();
    glTranslatef(dashboard_pos[0], dashboard_pos[1], dashboard_pos[2]);

    // 绘制外圈（仪表盘圆形）
    glColor3f(0.1f, 0.1f, 0.1f);  // 深灰色边框
    drawCircle(0.6f, 100);

    // 绘制刻度线（0, 2, 4, 6, 8, 10 共6个刻度）
    glColor3f(1.0f, 1.0f, 1.0f);  // 白色刻度线
    drawTicks(0.5f, 10);  // 总共绘制10个刻度

    // 绘制速度指针
    float pointerAngle = (90.0f - (180.0f * speed_ratio)) * M_PI / 180.0f;  // 根据车速计算角度
    glColor3f(1.0f, 0.0f, 0.0f);  // 红色指针
    drawPointer(pointerAngle);

    // 绘制刻度数字（0, 1, 2, ..., 10）
    for (int i = 0; i <= 10; i++) {
        float angle = (90.0f - 18.0f * i) * M_PI / 180.0f;
        drawNumber(0.45f, i, angle);
    }

    // 绘制"km/h"单位
    glColor3f(0.9f, 0.9f, 0.9f);
    glRasterPos2f(0.0f, -0.7f);
    const char* unit = "km/h";
    for (int i = 0; unit[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, unit[i]);
    }

    // 绘制当前速度值
    glColor3f(0.9f, 0.9f, 0.9f);
    char speedStr[50];
    snprintf(speedStr, sizeof(speedStr), "%.1f", speed);
    glRasterPos2f(-0.1f, 0.0f);
    for (int i = 0; speedStr[i] != '\0'; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, speedStr[i]);
    }

    // 恢复状态
    glPopMatrix();
    glutSwapBuffers();
}


// ------- Residuals for simple_car task ------
//     Position: Car should reach goal position (x, y)
//     Control:  Controls should be small
// ------------------------------------------
void SimpleCar::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                     double* residual) const {
  // ---------- Position (x, y) ----------
  // Goal position from mocap body
  residual[0] = data->qpos[0] - data->mocap_pos[0];  // x position
  residual[1] = data->qpos[1] - data->mocap_pos[1];  // y position

  // ---------- Control ----------
  residual[2] = data->ctrl[0];  // forward control
  residual[3] = data->ctrl[1];  // turn control
}

// -------- Transition for simple_car task --------
//   If car is within tolerance of goal ->
//   move goal randomly.
// ------------------------------------------------
void SimpleCar::TransitionLocked(mjModel* model, mjData* data) {
  // Car position (x, y)
  double car_pos[2] = {data->qpos[0], data->qpos[1]};
  
  // Goal position from mocap
  double goal_pos[2] = {data->mocap_pos[0], data->mocap_pos[1]};
  
  // Distance to goal
  double car_to_goal[2];
  mju_sub(car_to_goal, goal_pos, car_pos, 2);
  
  // If within tolerance, move goal to random position
  if (mju_norm(car_to_goal, 2) < 0.2) {
    absl::BitGen gen_;
    data->mocap_pos[0] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[1] = absl::Uniform<double>(gen_, -2.0, 2.0);
    data->mocap_pos[2] = 0.01;  // keep z at ground level
  }
}

// draw task-related geometry in the scene
// 改进后的立式仪表盘，放置在汽车正上方

void SimpleCar::ModifyScene(const mjModel* model, const mjData* data,
                            mjvScene* scene) const {
    if (!model || !data || !scene || !scene->geoms || scene->maxgeom <= 0) {
      return;
    }

    //打印车子信息
    // ===== 原地打印：位置 / 速度 / 加速度 / 转速条 =====
// ===== 油耗统计（累计）=====
    static double fuel_capacity = 100.0;   // 满油 = 100 单位
    static double fuel_used = 0.0;    // 累计油耗（任意单位）

// 1. 位置
    double pos_x = data->qpos[0];
    double pos_y = data->qpos[1];

// 2. 速度
    double vel_x = data->qvel[0];
    double vel_y = data->qvel[1];

// 3. 加速度
    double acc_x = data->qacc[0];
    double acc_y = data->qacc[1];

// 4. 车体速度（用于转速）
    double* car_velocity = SensorByName(model, data, "car_velocity");
    double speed_ms = car_velocity ? mju_norm3(car_velocity) : 0.0;

// 5. 转速条（30 个 #）
    const int BAR_LEN = 30;
    const double max_speed_ref = 5.0;   // 参考最大速度
    double rpm_ratio = speed_ms / max_speed_ref;
    if (rpm_ratio > 1.0) rpm_ratio = 1.0;
    if (rpm_ratio < 0.0) rpm_ratio = 0.0;

    int filled = static_cast<int>(rpm_ratio * BAR_LEN);

    char rpm_bar[BAR_LEN + 1];
    for (int i = 0; i < BAR_LEN; i++) {
        rpm_bar[i] = (i < filled) ? '#' : ' ';
    }
    rpm_bar[BAR_LEN] = '\0';


    double dt = model->opt.timestep;

// 油门控制（前进控制）
    double throttle = data->ctrl[0];

// 油耗系数（可在实验中说明）
    const double fuel_coeff = 0.2;

// 累计油耗
    fuel_used += fuel_coeff * std::abs(throttle) * dt;

// 不允许超过油箱容量
    if (fuel_used > fuel_capacity) {
        fuel_used = fuel_capacity;
    }
    double fuel_left = fuel_capacity - fuel_used;
    double fuel_percent = (fuel_left / fuel_capacity) * 100.0;

// 防止数值异常
    if (fuel_percent < 0.0) fuel_percent = 0.0;
    if (fuel_percent > 100.0) fuel_percent = 100.0;



// 6. 原地打印（注意：%s 对应 rpm_bar）
    printf(
            "\rPos(%.2f, %.2f) | "
            "Vel(%.2f, %.2f) | "
            "Acc(%.2f, %.2f) | "
            "Fuel %3.0f%%"
            "RPM [%s",
            pos_x, pos_y,
            vel_x, vel_y,
            acc_x, acc_y,
            fuel_percent,
            rpm_bar
    );

// 强制刷新
    fflush(stdout);




    // 获取汽车车身ID
  int car_body_id = mj_name2id(model, mjOBJ_BODY, "car");
  if (car_body_id < 0) return;  // 汽车车身未找到
  
  // 从传感器获取汽车线速度

  if (!car_velocity) return;  // 传感器未找到
  
  // 计算速度（速度向量的大小）

    double speed_kmh = speed_ms * 3.6;  // 将m/s转换为km/h

    double wheel_rpm = 0.0;
    {
      int left_joint_id = mj_name2id(model, mjOBJ_JOINT, "left");
      int right_joint_id = mj_name2id(model, mjOBJ_JOINT, "right");
      double omega = 0.0;
      int count = 0;
      if (left_joint_id >= 0) {
        int dof = model->jnt_dofadr[left_joint_id];
        if (dof >= 0 && dof < model->nv) {
          omega += std::abs(static_cast<double>(data->qvel[dof]));
          count++;
        }
      }
      if (right_joint_id >= 0) {
        int dof = model->jnt_dofadr[right_joint_id];
        if (dof >= 0 && dof < model->nv) {
          omega += std::abs(static_cast<double>(data->qvel[dof]));
          count++;
        }
      }
      if (count > 0) {
        omega /= count;
      } else if (double* car_angular_velocity =
                     SensorByName(model, data, "car_angular_velocity")) {
        omega = mju_norm3(car_angular_velocity);
      }
      wheel_rpm = omega * 60.0 / (2.0 * 3.141592653589793);
    }
  
  // 获取汽车位置
  double* car_pos = data->xpos + 3 * car_body_id;
  
  // 仪表盘位置（汽车正前方，立起来）
  float dashboard_pos[3] = {
    static_cast<float>(car_pos[0]),
    static_cast<float>(car_pos[1] ),  // 汽车前方0.5米
    static_cast<float>(car_pos[2] + 0.3f)   // 地面上方0.3米
  };
    const float gauge_scale = 2.0f;  // 仪表盘整体放大 2 倍（直径 ×2）

    const float bg_rgba[4] = {0.02f, 0.03f, 0.05f, 0.72f};
    const float ring_rgba[4] = {0.25f, 0.70f, 1.00f, 0.28f};

    auto rainbow_rgba = [&](float t, float alpha, float out_rgba[4]) {
      if (t < 0.0f) t = 0.0f;
      if (t > 1.0f) t = 1.0f;

      float r = 0.0f, g = 0.0f, b = 0.0f;
      if (t < 0.33f) {
        const float u = t / 0.33f;
        r = 0.12f;
        g = 0.55f + 0.45f * u;
        b = 1.00f - 0.30f * u;
      } else if (t < 0.66f) {
        const float u = (t - 0.33f) / 0.33f;
        r = 0.12f + 0.88f * u;
        g = 1.00f - 0.10f * u;
        b = 0.70f - 0.55f * u;
      } else {
        const float u = (t - 0.66f) / 0.34f;
        r = 1.00f;
        g = 0.90f - 0.75f * u;
        b = 0.15f;
      }

      out_rgba[0] = r;
      out_rgba[1] = g;
      out_rgba[2] = b;
      out_rgba[3] = alpha;
    };

    auto add_geom = [&](mjtGeom type, const mjtNum size[3], const mjtNum pos[3],
                        const mjtNum mat[9], const float rgba[4]) -> mjvGeom* {
      if (scene->ngeom >= scene->maxgeom) return nullptr;
      mjvGeom* geom = scene->geoms + scene->ngeom;
      mjv_initGeom(geom, type, size, pos, mat, rgba);
      geom->category = mjCAT_DECOR;
      scene->ngeom++;
      return geom;
    };


    // 最大速度参考值（km/h），根据要求是0-10
  const float max_speed_kmh = 10.0f;
  
  // 速度百分比（0-1）
  float speed_ratio = static_cast<float>(speed_kmh) / max_speed_kmh;
  if (speed_ratio > 1.0f) speed_ratio = 1.0f;
  
  // 仪表盘旋转矩阵（绕X轴旋转90度，再顺时针旋转90度）
  double angle_x = 90.0 * 3.14159 / 180.0;  // 绕X轴旋转90度（立起来）
  double cos_x = cos(angle_x);
  double sin_x = sin(angle_x);
  double mat_x[9] = {
    1, 0,      0,
    0, cos_x, -sin_x,
    0, sin_x,  cos_x
  };
  
  double angle_z = -90.0 * 3.14159 / 180.0;  // 绕Z轴旋转-90度（顺时针）
  double cos_z = cos(angle_z);
  double sin_z = sin(angle_z);
  double mat_z[9] = {
    cos_z, -sin_z, 0,
    sin_z,  cos_z, 0,
    0,      0,     1
  };
  
  // 组合旋转矩阵：先绕X轴旋转90°，再绕Z轴顺时针旋转90°
  double dashboard_rot_mat[9];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      dashboard_rot_mat[i*3 + j] = 0;
      for (int k = 0; k < 3; k++) {
        dashboard_rot_mat[i*3 + j] += mat_z[i*3 + k] * mat_x[k*3 + j];
      }
    }
  }
  
  mjtNum dash_pos_num[3] = {static_cast<mjtNum>(dashboard_pos[0]),
                            static_cast<mjtNum>(dashboard_pos[1]),
                            static_cast<mjtNum>(dashboard_pos[2])};
  mjtNum dash_mat_num[9];
  for (int j = 0; j < 9; j++) dash_mat_num[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);

  {
    mjtNum size[3] = {0.145 * gauge_scale, 0.010 * gauge_scale, 0};
    add_geom(mjGEOM_CYLINDER, size, dash_pos_num, dash_mat_num, bg_rgba);
  }

  {
    mjtNum size[3] = {0.152 * gauge_scale, 0.012 * gauge_scale, 0};
    add_geom(mjGEOM_CYLINDER, size, dash_pos_num, dash_mat_num, ring_rgba);
  }

  {
    mjtNum size[3] = {0.060, 0.060, 0.060};
    mjtNum pos[3] = {dash_pos_num[0], dash_pos_num[1],
                     static_cast<mjtNum>(dashboard_pos[2] + 0.125f * gauge_scale)};
    const float rgba[4] = {0.86f, 0.88f, 0.95f, 1.0f};
    mjvGeom* geom = add_geom(mjGEOM_LABEL, size, pos, dash_mat_num, rgba);
    if (geom) {
      std::strncpy(geom->label, "SPEED", sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
    }
  }

  {
    const int kSegCount = 24;
    const float seg_radius = 0.134f * gauge_scale;
    const float seg_width = 0.0042f * gauge_scale;
    const float seg_half_len = 0.012f * gauge_scale;
    const float seg_depth = 0.0032f * gauge_scale;

    for (int i = 0; i < kSegCount; i++) {
      if (scene->ngeom >= scene->maxgeom) break;

      const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(kSegCount);
      const float angle_deg = 180.0f - 180.0f * t;
      const float rad_angle = angle_deg * 3.14159f / 180.0f;

      const float seg_y = dashboard_pos[1] - seg_radius * cos(rad_angle);
      const float seg_z = dashboard_pos[2] + seg_radius * sin(rad_angle);

      const double rot_deg = static_cast<double>(angle_deg) - 90.0;
      const double rot_rad = rot_deg * 3.14159 / 180.0;
      const double cos_t = cos(rot_rad);
      const double sin_t = sin(rot_rad);

      const double seg_rot_mat[9] = {cos_t, -sin_t, 0, sin_t, cos_t, 0, 0, 0, 1};
      double seg_mat[9];
      for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
          seg_mat[r * 3 + c] = 0;
          for (int k = 0; k < 3; k++) {
            seg_mat[r * 3 + c] +=
                dashboard_rot_mat[r * 3 + k] * seg_rot_mat[k * 3 + c];
          }
        }
      }

      mjtNum size[3] = {seg_width, seg_half_len, seg_depth};
      mjtNum pos[3] = {dash_pos_num[0], static_cast<mjtNum>(seg_y),
                       static_cast<mjtNum>(seg_z)};
      mjtNum mat[9];
      for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(seg_mat[j]);

      const bool active = (t <= speed_ratio + 1e-6f);
      float r = 1.0f, g = 1.0f, b = 0.22f;
      if (t < 0.55f) {
        r = 0.20f + 0.80f * (t / 0.55f);
        g = 0.95f;
      } else {
        r = 1.0f;
        g = 0.95f - 0.80f * ((t - 0.55f) / 0.45f);
        if (g < 0.15f) g = 0.15f;
      }
      float rgba[4] = {active ? r : 0.20f, active ? g : 0.22f,
                       active ? b : 0.24f, active ? 0.88f : 0.18f};
      add_geom(mjGEOM_BOX, size, pos, mat, rgba);
    }
  }

    // 2. 添加刻度线（0~10 全刻度）
    const int kMaxTick = 10;
    const int kTickCount = kMaxTick + 1;

    for (int i = 0; i < kTickCount; i++) {
        if (scene->ngeom >= scene->maxgeom) break;

        int tick_value = i;

        // 角度：0 在左(180°)，10 在右(0°)
        float tick_angle_deg = 180.0f - (180.0f * tick_value / kMaxTick);
        float rad_tick_angle = tick_angle_deg * 3.14159f / 180.0f;

        // ——【新增】刻度长度（必须有）——
        float full_len = ((tick_value % 5 == 0) ? 0.034f : 0.020f) * gauge_scale;
        float half_len = full_len * 0.5f;




        float tick_radius_outer = 0.135f * gauge_scale;
        float tick_radius_center = tick_radius_outer - half_len;

        float tick_y = dashboard_pos[1] - tick_radius_center * cos(rad_tick_angle);
        float tick_z = dashboard_pos[2] + tick_radius_center * sin(rad_tick_angle);

        // ---- 刻度指向圆心 ----
        double tick_rot_angle = tick_angle_deg - 90.0;
        double rad_tick_rot = tick_rot_angle * 3.14159 / 180.0;
        double cos_t = cos(rad_tick_rot);
        double sin_t = sin(rad_tick_rot);

        double tick_rot_mat[9] = {
                cos_t, -sin_t, 0,
                sin_t,  cos_t, 0,
                0,      0,     1
        };

        double tick_mat[9];
        for (int r = 0; r < 3; r++) {
            for (int c = 0; c < 3; c++) {
                tick_mat[r*3 + c] = 0;
                for (int k = 0; k < 3; k++) {
                    tick_mat[r*3 + c] += dashboard_rot_mat[r*3 + k] * tick_rot_mat[k*3 + c];
                }
            }
        }

        {
          const bool major = (tick_value % 5 == 0);
          float tick_rgba[4];
          rainbow_rgba(static_cast<float>(tick_value) /
                           static_cast<float>(kMaxTick),
                       major ? 0.95f : 0.75f, tick_rgba);
          mjtNum size[3] = {static_cast<mjtNum>((major ? 0.0042f : 0.0032f) *
                                                gauge_scale),
                            static_cast<mjtNum>(half_len),
                            static_cast<mjtNum>((major ? 0.0042f : 0.0032f) *
                                                gauge_scale)};
          mjtNum pos[3] = {dash_pos_num[0], static_cast<mjtNum>(tick_y),
                           static_cast<mjtNum>(tick_z)};
          mjtNum mat[9];
          for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(tick_mat[j]);
          add_geom(mjGEOM_BOX, size, pos, mat, tick_rgba);
        }

        // ---- 数字标签 ----
        if (scene->ngeom >= scene->maxgeom) break;

        mjvGeom* label_geom = scene->geoms + scene->ngeom;
        float label_radius = 0.18f * gauge_scale;

        const bool major = (tick_value % 5 == 0);
        float label_rgba[4];
        rainbow_rgba(static_cast<float>(tick_value) /
                         static_cast<float>(kMaxTick),
                     major ? 1.0f : 0.92f, label_rgba);
        mjtNum size[3] = {static_cast<mjtNum>((major ? 0.060f : 0.048f) *
                                              gauge_scale),
                          static_cast<mjtNum>((major ? 0.060f : 0.048f) *
                                              gauge_scale),
                          static_cast<mjtNum>((major ? 0.060f : 0.048f) *
                                              gauge_scale)};
        mjtNum pos[3] = {static_cast<mjtNum>(dashboard_pos[0]),
                         static_cast<mjtNum>(dashboard_pos[1] -
                                             label_radius * cos(rad_tick_angle)),
                         static_cast<mjtNum>(dashboard_pos[2] +
                                             label_radius * sin(rad_tick_angle))};
        mjtNum mat[9];
        for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);
        mjv_initGeom(label_geom, mjGEOM_LABEL, size, pos, mat, label_rgba);
        label_geom->category = mjCAT_DECOR;
        std::snprintf(label_geom->label, sizeof(label_geom->label), "%d", tick_value);
        scene->ngeom++;
    }

  // 3. 速度指针（改为红色）
  if (scene->ngeom < scene->maxgeom) {
    float angle = 180.0f - 180.0f * speed_ratio;
    float rad_angle = angle * 3.14159f / 180.0f;

    float pointer_y = dashboard_pos[1] - 0.0275f * gauge_scale * cos(rad_angle);
    float pointer_z = dashboard_pos[2] + 0.0275f * gauge_scale * sin(rad_angle);

    double pointer_angle = angle - 90.0;
    double rad_pointer_angle = pointer_angle * 3.14159 / 180.0;
    double cos_p = cos(rad_pointer_angle);
    double sin_p = sin(rad_pointer_angle);
    double pointer_rot_mat[9] = {
        cos_p, -sin_p, 0,
        sin_p,  cos_p, 0,
        0,      0,     1
    };

    double temp_mat[9];
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        temp_mat[i*3 + j] = 0;
        for (int k = 0; k < 3; k++) {
          temp_mat[i*3 + j] +=
              dashboard_rot_mat[i*3 + k] * pointer_rot_mat[k*3 + j];
        }
      }
    }

    mjvGeom* geom = scene->geoms + scene->ngeom;
    mjtNum size[3] = {0.0048 * gauge_scale, 0.060 * gauge_scale,
                      0.0036 * gauge_scale};
    mjtNum pos[3] = {static_cast<mjtNum>(dashboard_pos[0]),
                     static_cast<mjtNum>(pointer_y),
                     static_cast<mjtNum>(pointer_z)};
    mjtNum mat[9];
    for (int i = 0; i < 9; i++) mat[i] = static_cast<mjtNum>(temp_mat[i]);
    float rgba[4] = {1.0f, 0.55f, 0.18f, 1.0f};
    mjv_initGeom(geom, mjGEOM_BOX, size, pos, mat, rgba);
    geom->category = mjCAT_DECOR;
    scene->ngeom++;
  }
  
  // 4. 中心固定点（小圆点）
  {
    mjtNum size[3] = {0.0072 * gauge_scale, 0.0072 * gauge_scale,
                      0.0072 * gauge_scale};
    const float rgba[4] = {0.08f, 0.10f, 0.14f, 0.95f};
    add_geom(mjGEOM_SPHERE, size, dash_pos_num, dash_mat_num, rgba);
  }
  {
    mjtNum size[3] = {0.0042 * gauge_scale, 0.0042 * gauge_scale,
                      0.0042 * gauge_scale};
    const float rgba[4] = {0.90f, 0.92f, 0.98f, 0.95f};
    add_geom(mjGEOM_SPHERE, size, dash_pos_num, dash_mat_num, rgba);
  }
  
  // 5. 数字速度显示（在仪表盘中央偏上）
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    char speed_label[50];
    std::snprintf(speed_label, sizeof(speed_label), "%.1f", speed_kmh);
    mjtNum size[3] = {0.095, 0.095, 0.095};
    mjtNum pos[3] = {static_cast<mjtNum>(dashboard_pos[0]),
                     static_cast<mjtNum>(dashboard_pos[1]),
                     static_cast<mjtNum>(dashboard_pos[2] + 0.02f)};
    mjtNum mat[9];
    for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);
    float rgba[4] = {0.92f, 0.94f, 1.00f, 1.0f};
    mjv_initGeom(geom, mjGEOM_LABEL, size, pos, mat, rgba);
    geom->category = mjCAT_DECOR;
    std::strncpy(geom->label, speed_label, sizeof(geom->label) - 1);
    geom->label[sizeof(geom->label) - 1] = '\0';
    scene->ngeom++;
  }
  
  // 6. 添加"km/h"单位标签（在数字下方）
  if (scene->ngeom < scene->maxgeom) {
    mjvGeom* geom = scene->geoms + scene->ngeom;
    mjtNum size[3] = {0.05, 0.05, 0.05};
    mjtNum pos[3] = {static_cast<mjtNum>(dashboard_pos[0]),
                     static_cast<mjtNum>(dashboard_pos[1]),
                     static_cast<mjtNum>(dashboard_pos[2] - 0.06f)};
    mjtNum mat[9];
    for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);
    float rgba[4] = {0.70f, 0.74f, 0.82f, 1.0f};
    mjv_initGeom(geom, mjGEOM_LABEL, size, pos, mat, rgba);
    geom->category = mjCAT_DECOR;
    std::strncpy(geom->label, "km/h", sizeof(geom->label) - 1);
    geom->label[sizeof(geom->label) - 1] = '\0';
    scene->ngeom++;
  }

  auto draw_small_gauge = [&](const float center[3], float ratio,
                              const float pointer_rgba[4],
                              const char* title_text,
                              const char* value_text, double max_value) {
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    const float scale = gauge_scale * 0.70f;
    const float ring_radius = 0.10f * scale;
    const float ring_thickness = 0.015f * scale;
    const float pointer_width = 0.0042f * scale;
    const float pointer_half_len = 0.042f * scale;
    const float pointer_depth = 0.0034f * scale;

    mjtNum center_pos[3] = {static_cast<mjtNum>(center[0]),
                            static_cast<mjtNum>(center[1]),
                            static_cast<mjtNum>(center[2])};
    mjtNum center_mat[9];
    for (int j = 0; j < 9; j++) center_mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);

    {
      mjtNum size[3] = {ring_radius * 0.98f, ring_thickness * 0.55f, 0};
      add_geom(mjGEOM_CYLINDER, size, center_pos, center_mat, bg_rgba);
    }

    {
      float rgba[4] = {0.20f + 0.55f * pointer_rgba[0],
                       0.20f + 0.55f * pointer_rgba[1],
                       0.20f + 0.55f * pointer_rgba[2], 0.26f};
      mjtNum size[3] = {ring_radius * 1.05f, ring_thickness * 0.85f, 0};
      add_geom(mjGEOM_CYLINDER, size, center_pos, center_mat, rgba);
    }

    {
      const int kSegCount = 12;
      const float seg_radius = ring_radius * 0.88f;
      const float seg_width = 0.0036f * scale;
      const float seg_half_len = 0.0105f * scale;
      const float seg_depth = 0.0030f * scale;

      for (int i = 0; i < kSegCount; i++) {
        if (scene->ngeom >= scene->maxgeom) break;

        const float t = (static_cast<float>(i) + 0.5f) / static_cast<float>(kSegCount);
        const float angle_deg = 180.0f - 180.0f * t;
        const float rad_angle = angle_deg * 3.14159f / 180.0f;

        const float seg_y = center[1] - seg_radius * cos(rad_angle);
        const float seg_z = center[2] + seg_radius * sin(rad_angle);

        const double rot_deg = static_cast<double>(angle_deg) - 90.0;
        const double rot_rad = rot_deg * 3.14159 / 180.0;
        const double cos_t = cos(rot_rad);
        const double sin_t = sin(rot_rad);
        const double seg_rot_mat[9] = {cos_t, -sin_t, 0, sin_t, cos_t, 0, 0, 0, 1};

        double seg_mat[9];
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 3; c++) {
            seg_mat[r * 3 + c] = 0;
            for (int k = 0; k < 3; k++) {
              seg_mat[r * 3 + c] +=
                  dashboard_rot_mat[r * 3 + k] * seg_rot_mat[k * 3 + c];
            }
          }
        }

        mjtNum size[3] = {seg_width, seg_half_len, seg_depth};
        mjtNum pos[3] = {center_pos[0], static_cast<mjtNum>(seg_y),
                         static_cast<mjtNum>(seg_z)};
        mjtNum mat[9];
        for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(seg_mat[j]);

        const bool active = (t <= ratio + 1e-6f);
        float rgba[4] = {active ? pointer_rgba[0] : 0.20f,
                         active ? pointer_rgba[1] : 0.22f,
                         active ? pointer_rgba[2] : 0.24f,
                         active ? 0.88f : 0.18f};
        add_geom(mjGEOM_BOX, size, pos, mat, rgba);
      }
    }

    {
      const int kMaxTick = 10;
      const int kTickCount = kMaxTick + 1;
      const float tick_radius_outer = ring_radius * 1.03f;
      const float label_radius = ring_radius * 1.30f;

      for (int i = 0; i < kTickCount; i++) {
        if (scene->ngeom >= scene->maxgeom) break;

        const bool major = (i % 5 == 0);
        const float tick_angle_deg =
            180.0f - (180.0f * static_cast<float>(i) / static_cast<float>(kMaxTick));
        const float rad_tick_angle = tick_angle_deg * 3.14159f / 180.0f;

        const float full_len = (major ? 0.020f : 0.013f) * scale;
        const float half_len = full_len * 0.5f;
        const float tick_radius_center = tick_radius_outer - half_len;

        const float tick_y = center[1] - tick_radius_center * cos(rad_tick_angle);
        const float tick_z = center[2] + tick_radius_center * sin(rad_tick_angle);

        const double tick_rot_angle = static_cast<double>(tick_angle_deg) - 90.0;
        const double rad_tick_rot = tick_rot_angle * 3.14159 / 180.0;
        const double cos_t = cos(rad_tick_rot);
        const double sin_t = sin(rad_tick_rot);

        const double tick_rot_mat[9] = {cos_t, -sin_t, 0, sin_t, cos_t, 0, 0, 0, 1};
        double tick_mat[9];
        for (int r = 0; r < 3; r++) {
          for (int c = 0; c < 3; c++) {
            tick_mat[r * 3 + c] = 0;
            for (int k = 0; k < 3; k++) {
              tick_mat[r * 3 + c] +=
                  dashboard_rot_mat[r * 3 + k] * tick_rot_mat[k * 3 + c];
            }
          }
        }

        mjtNum size[3] = {static_cast<mjtNum>((major ? 0.0038f : 0.0030f) * scale),
                          static_cast<mjtNum>(half_len),
                          static_cast<mjtNum>((major ? 0.0038f : 0.0030f) * scale)};
        mjtNum pos[3] = {center_pos[0], static_cast<mjtNum>(tick_y),
                         static_cast<mjtNum>(tick_z)};
        mjtNum mat[9];
        for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(tick_mat[j]);
        float tick_rgba[4];
        rainbow_rgba(static_cast<float>(i) /
                         static_cast<float>(kMaxTick),
                     major ? 0.90f : 0.65f, tick_rgba);
        add_geom(mjGEOM_BOX, size, pos, mat, tick_rgba);

        if (!major) continue;
        if (scene->ngeom >= scene->maxgeom) break;

        mjvGeom* label_geom = scene->geoms + scene->ngeom;
        const double tick_value =
            (static_cast<double>(i) / static_cast<double>(kMaxTick)) * max_value;
        const int tick_value_int = static_cast<int>(std::lround(tick_value));

        mjtNum label_size[3] = {0.032, 0.032, 0.032};
        mjtNum label_pos[3] = {center_pos[0],
                               static_cast<mjtNum>(center[1] -
                                                   label_radius * cos(rad_tick_angle)),
                               static_cast<mjtNum>(center[2] +
                                                   label_radius * sin(rad_tick_angle))};
        mjtNum label_mat[9];
        for (int j = 0; j < 9; j++) label_mat[j] = center_mat[j];
        float label_rgba[4];
        rainbow_rgba(static_cast<float>(i) /
                         static_cast<float>(kMaxTick),
                     0.92f, label_rgba);
        mjv_initGeom(label_geom, mjGEOM_LABEL, label_size, label_pos, label_mat,
                    label_rgba);
        label_geom->category = mjCAT_DECOR;
        std::snprintf(label_geom->label, sizeof(label_geom->label), "%d",
                      tick_value_int);
        scene->ngeom++;
      }
    }

    if (scene->ngeom < scene->maxgeom) {
      float angle = 180.0f - 180.0f * ratio;
      float rad_angle = angle * 3.14159f / 180.0f;
      float pointer_y =
          center[1] - (pointer_half_len * 0.5f) * cos(rad_angle);
      float pointer_z =
          center[2] + (pointer_half_len * 0.5f) * sin(rad_angle);

      double pointer_angle = static_cast<double>(angle) - 90.0;
      double rad_pointer_angle = pointer_angle * 3.14159 / 180.0;
      double cos_p = cos(rad_pointer_angle);
      double sin_p = sin(rad_pointer_angle);
      double pointer_rot_mat[9] = {
          cos_p, -sin_p, 0,
          sin_p,  cos_p, 0,
          0,      0,     1
      };

      double temp_mat[9];
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
          temp_mat[i * 3 + j] = 0;
          for (int k = 0; k < 3; k++) {
            temp_mat[i * 3 + j] +=
                dashboard_rot_mat[i * 3 + k] * pointer_rot_mat[k * 3 + j];
          }
        }
      }

      mjvGeom* geom = scene->geoms + scene->ngeom;
      mjtNum size[3] = {pointer_width, pointer_half_len, pointer_depth};
      mjtNum pos[3] = {static_cast<mjtNum>(center[0]),
                       static_cast<mjtNum>(pointer_y),
                       static_cast<mjtNum>(pointer_z)};
      mjtNum mat[9];
      for (int i = 0; i < 9; i++) mat[i] = static_cast<mjtNum>(temp_mat[i]);
      float rgba[4] = {pointer_rgba[0], pointer_rgba[1], pointer_rgba[2],
                       pointer_rgba[3]};
      mjv_initGeom(geom, mjGEOM_BOX, size, pos, mat, rgba);
      geom->category = mjCAT_DECOR;
      scene->ngeom++;
    }

    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      mjtNum size[3] = {0.05, 0.05, 0.05};
      mjtNum pos[3] = {static_cast<mjtNum>(center[0]),
                       static_cast<mjtNum>(center[1]),
                       static_cast<mjtNum>(center[2] + 0.10f * scale)};
      mjtNum mat[9];
      for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);
      float rgba[4] = {0.80f, 0.83f, 0.90f, 1.0f};
      mjv_initGeom(geom, mjGEOM_LABEL, size, pos, mat, rgba);
      geom->category = mjCAT_DECOR;
      std::strncpy(geom->label, title_text, sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }

    if (scene->ngeom < scene->maxgeom) {
      mjvGeom* geom = scene->geoms + scene->ngeom;
      mjtNum size[3] = {0.070, 0.070, 0.070};
      mjtNum pos[3] = {static_cast<mjtNum>(center[0]),
                       static_cast<mjtNum>(center[1]),
                       static_cast<mjtNum>(center[2] - 0.09f * scale)};
      mjtNum mat[9];
      for (int j = 0; j < 9; j++) mat[j] = static_cast<mjtNum>(dashboard_rot_mat[j]);
      float rgba[4] = {0.92f, 0.94f, 1.00f, 1.0f};
      mjv_initGeom(geom, mjGEOM_LABEL, size, pos, mat, rgba);
      geom->category = mjCAT_DECOR;
      std::strncpy(geom->label, value_text, sizeof(geom->label) - 1);
      geom->label[sizeof(geom->label) - 1] = '\0';
      scene->ngeom++;
    }
  };

  const float side_offset =
      (0.18f + 0.12f + 0.06f) * gauge_scale;

  float fuel_center[3] = {dashboard_pos[0], dashboard_pos[1] + side_offset,
                          dashboard_pos[2]};
  float rpm_center[3] = {dashboard_pos[0], dashboard_pos[1] - side_offset,
                         dashboard_pos[2]};

  char fuel_value_text[32];
  std::snprintf(fuel_value_text, sizeof(fuel_value_text), "%3.0f%%",
                fuel_percent);
  const float fuel_ratio = static_cast<float>(fuel_percent / 100.0);
  const float fuel_pointer_rgba[4] = {0.20f, 0.95f, 0.40f, 1.0f};
  draw_small_gauge(fuel_center, fuel_ratio, fuel_pointer_rgba, "FUEL",
                   fuel_value_text, 100.0);

  const double rpm_max = 600.0;
  float rpm_ratio_gauge = static_cast<float>(wheel_rpm / rpm_max);
  if (rpm_ratio_gauge < 0.0f) rpm_ratio_gauge = 0.0f;
  if (rpm_ratio_gauge > 1.0f) rpm_ratio_gauge = 1.0f;
  char rpm_value_text[32];
  std::snprintf(rpm_value_text, sizeof(rpm_value_text), "%.0f",
                wheel_rpm);
  const float rpm_pointer_rgba[4] = {0.95f, 0.30f, 0.90f, 1.0f};
  draw_small_gauge(rpm_center, rpm_ratio_gauge, rpm_pointer_rgba, "RPM",
                   rpm_value_text, rpm_max);
}

}  // namespace mjpc
