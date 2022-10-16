#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <string.h>
#include <vector>

const int WINDOW_WIDTH(1280);
const int WINDOW_HEIGHT(720);
const char* WINDOW_NAME("Spatial Data Structures");

const int TARGET_FPS(60);
const float TIMESTEP(1.0f / TARGET_FPS);

const KeyboardKey SPAWN_KEY(KEY_SPACE);

enum CircleSize { small = 0, big = 1 };

const int SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY(1);
const int SMALL_CIRCLE_RADIUS_MIN(5);
const int SMALL_CIRCLE_RADIUS_MAX(10);
const int SMALL_CIRCLE_MASS(1);
const float SMALL_CIRCLE_VELOCITY_MIN(5.0f);
const float SMALL_CIRCLE_VELOCITY_MAX(100.0f);

const int NUMBER_OF_PRESSES_UNTIL_BIG_CIRCLE_SPAWNS(10);
const int BIG_CIRCLE_RADIUS(25);
const int BIG_CIRCLE_MASS(10);

const float FRICTION(-0.75f);
const float VELOCITY_THRESHOLD(5.0f);
const float ELASTICITY(0.5f);

// https://cplusplus.com/forum/beginner/81180/
// Returns a random float within min and max
static float randf(const float min, const float max) {
  float result =
    (rand() / static_cast<float>(RAND_MAX) * (max - min + 1)) + min;
  return result;
}

// Returns 1 or -1
static int directionMultiplier() {
  int randomNumber = rand() % 100;  // 0 to 99
  if (randomNumber < 50) {
    return -1.0f;
  }
  return 1.0f;
}

struct Circle {
  int radius;
  int mass;
  Color color;

  Vector2 acceleration;
  Vector2 velocity;
  Vector2 position;

	Circle() {}

  // If big, spawn at bottom middle of screen
  // Else, spawn at middle
  void spawn(const CircleSize size = small) {
    color = {
      static_cast<unsigned char>(rand() % 256),
      static_cast<unsigned char>(rand() % 256),
      static_cast<unsigned char>(rand() % 256), 255};
    velocity = {
      randf(SMALL_CIRCLE_VELOCITY_MIN, SMALL_CIRCLE_VELOCITY_MAX) *
        directionMultiplier(),
      randf(SMALL_CIRCLE_VELOCITY_MIN, SMALL_CIRCLE_VELOCITY_MAX) *
        directionMultiplier()};
    position = {WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2};
    radius = rand() % (SMALL_CIRCLE_RADIUS_MAX - SMALL_CIRCLE_RADIUS_MIN) +
             SMALL_CIRCLE_RADIUS_MIN;
    mass = SMALL_CIRCLE_MASS;
  }

  void draw() { DrawCircle(position.x, position.y, radius, color); }

  void update(Vector2 force = {0.0f, 0.0f}, float timestep = TIMESTEP) {
    // acceleration = Vector2Add(
    //   Vector2Scale(force, 1 / mass), (Vector2Scale(velocity, FRICTION))
    // ); // With friction
    acceleration = Vector2Scale(force, 1 / mass);  // No friction
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, TIMESTEP));
    velocity.x = (abs(velocity.x) < VELOCITY_THRESHOLD) ? 0.0f : velocity.x;
    velocity.y = (abs(velocity.y) < VELOCITY_THRESHOLD) ? 0.0f : velocity.y;
    position = Vector2Add(position, Vector2Scale(velocity, TIMESTEP));
  }

  static float getImpulse(
    Circle a, Circle b, Vector2 relativeVelocity, Vector2 collisionNormal
  ) {
    float impulse(-(
      ((1.0f + ELASTICITY) *
       (Vector2DotProduct(relativeVelocity, collisionNormal)) /
       (Vector2DotProduct(collisionNormal, collisionNormal) *
        ((1.0f / a.mass) + (1.0f / b.mass))))
    ));

    return impulse;
  }
};

int main() {
  srand(GetTime());

  int numberOfSmallCirclesPresent = 0;
  
	// Counts the number of times the user has spawned 10 small circles
  int numberOfSpawnKeyPresses = 0;
	
  std::vector<Circle> smallCircles;

  char smallCircleCountBuffer[10];
  int numberOfSmallCirclesPresentFormatted;

  float accumulator(0.0f);
  float deltaTime(0.0f);

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME);
  SetTargetFPS(TARGET_FPS);
  while (!WindowShouldClose()) {
    deltaTime = GetFrameTime();

    if (IsKeyPressed(SPAWN_KEY)) {
      numberOfSpawnKeyPresses += 1;
      // Spawn 10 small circles
      int numberOfSmallCirclesAfterSpawning =
        numberOfSmallCirclesPresent + SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY;
      for (size_t i = numberOfSmallCirclesPresent;
           i < numberOfSmallCirclesAfterSpawning; i++) {
				smallCircles.push_back(Circle());
        smallCircles[i].spawn();
      }
      numberOfSmallCirclesPresent = numberOfSmallCirclesAfterSpawning;
    }

    // Physics update
    accumulator += deltaTime;
    while (accumulator >= TIMESTEP) {
      for (size_t i = 0; i < numberOfSmallCirclesPresent; i++) {
        Circle* currentCircle = &smallCircles[i];
        // Check if the circle should bounce off of the screen edge
        bool circleIsOutOfBoundsX =
          currentCircle->position.x >= (WINDOW_WIDTH - currentCircle->radius) ||
          currentCircle->position.x <= currentCircle->radius;
        bool circleIsOutOfBoundsY =
          currentCircle->position.y >=
            (WINDOW_HEIGHT - currentCircle->radius) ||
          currentCircle->position.y <= currentCircle->radius;
        if (circleIsOutOfBoundsX) {
          currentCircle->velocity.x *= -1.0f;
        }
        if (circleIsOutOfBoundsY) {
          currentCircle->velocity.y *= -1.0f;
        }

        currentCircle->update();
      }
      accumulator -= TIMESTEP;
    }

    // Draw
    BeginDrawing();
    ClearBackground(WHITE);

    for (size_t i = 0; i < numberOfSmallCirclesPresent; i++) {
      smallCircles[i].draw();
    }

    // Small Circle Counter
    numberOfSmallCirclesPresentFormatted =
      sprintf(smallCircleCountBuffer, "%d\n", numberOfSmallCirclesPresent);
    DrawText(smallCircleCountBuffer, 10, 10, 30, BLACK);

    EndDrawing();
  }

  return 0;
}