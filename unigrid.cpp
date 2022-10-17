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

const float CIRCLE_VELOCITY_MIN(5.0f);
const float CIRCLE_VELOCITY_MAX(100.0f);

const int SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY(25);
const int SMALL_CIRCLE_RADIUS_MIN(5);
const int SMALL_CIRCLE_RADIUS_MAX(10);
const int SMALL_CIRCLE_MASS(1);

const int NUMBER_OF_PRESSES_UNTIL_BIG_CIRCLE_SPAWNS(10);
const int BIG_CIRCLE_RADIUS(25);
const int BIG_CIRCLE_MASS(10);

const float FRICTION(-0.1f);
const float VELOCITY_THRESHOLD(5.0f);
const float ELASTICITY(0.5f);

const int GRID_SIZE(40);

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
    velocity.x =
      randf(CIRCLE_VELOCITY_MIN, CIRCLE_VELOCITY_MAX) * directionMultiplier();

    if (size == CircleSize::small) {
      radius = rand() % (SMALL_CIRCLE_RADIUS_MAX - SMALL_CIRCLE_RADIUS_MIN) +
               SMALL_CIRCLE_RADIUS_MIN;
      mass = SMALL_CIRCLE_MASS;
      setPosition({WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2});
      velocity.y =
        randf(CIRCLE_VELOCITY_MIN, CIRCLE_VELOCITY_MAX) * directionMultiplier();
    } else {
      radius = BIG_CIRCLE_RADIUS;
      mass = BIG_CIRCLE_MASS;
      setPosition({WINDOW_WIDTH / 2, WINDOW_HEIGHT - (float)radius});
      velocity.y = randf(CIRCLE_VELOCITY_MIN, CIRCLE_VELOCITY_MAX);
    }
  }

  void draw() { DrawCircle(position.x, position.y, radius, color); }

  // Update physics and gridPosition
  void update(
    const Vector2 force = {0.0f, 0.0f}, const float timestep = TIMESTEP
  ) {
    acceleration = Vector2Add(
      Vector2Scale(force, 1 / mass), (Vector2Scale(velocity, FRICTION))
    );
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, TIMESTEP));
    velocity.x = (abs(velocity.x) < VELOCITY_THRESHOLD) ? 0.0f : velocity.x;
    velocity.y = (abs(velocity.y) < VELOCITY_THRESHOLD) ? 0.0f : velocity.y;
    setPosition(Vector2Add(position, Vector2Scale(velocity, TIMESTEP)));
  }

  // If idx is -1, double-checking collision will happen
  void handleCircleCollision(
    const std::vector<Circle> circles, const size_t idx = -1
  ) {
    // Choose if the loop should start at 0 or at idx
    size_t iterator = (idx == -1) ? 0 : idx;
    for (size_t i = iterator; i < circles.size(); i++) {
      Circle* a = this;
      Circle b = circles[i];

      if (a == &b) continue;

      // If a and b don't share any gridPosition, continue loop
      // if (!sharesGridPositions(*a, b)) continue;

      float sumOfRadii(pow(a->radius + b.radius, 2));
      float distanceBetweenCenters(Vector2DistanceSqr(a->position, b.position));

      // Collision detected
      if (sumOfRadii >= distanceBetweenCenters) {
        Vector2 collisionNormalAB(
          {b.position.x - a->position.x, b.position.y - a->position.y}
        );
        Vector2 relativeVelocityAB(Vector2Subtract(a->velocity, b.velocity));
        Vector2 collisionNormalABNormalized(Vector2Normalize(collisionNormalAB)
        );
        Vector2 relativeVelocityABNormalized(Vector2Normalize(relativeVelocityAB
        ));

        // I think we should also separate balls that are touching
        if (Vector2Length(relativeVelocityAB) <= 0.1f) {
          a->setPosition(Vector2Subtract(
            a->position, Vector2Scale(collisionNormalABNormalized, 0.5f)
          ));
          b.setPosition(Vector2Add(
            b.position, Vector2Scale(collisionNormalABNormalized, 0.5f)
          ));
        }

        // Collision response
        // Check dot product between collision normal and relative velocity
        if (Vector2DotProduct(relativeVelocityABNormalized, collisionNormalABNormalized) > 0) {
          float impulse =
            Circle::getImpulse(*a, b, relativeVelocityAB, collisionNormalAB);
          a->velocity = Vector2Add(
            a->velocity,
            Vector2Scale(
              Vector2Scale(collisionNormalAB, 1.0f / a->mass), impulse
            )
          );
          b.velocity = Vector2Subtract(
            b.velocity,
            Vector2Scale(
              Vector2Scale(collisionNormalAB, 1.0f / b.mass), impulse
            )
          );
        }
      }
    }
  }

  void handleEdgeCollision(
    const int screenWidth = WINDOW_WIDTH, const int screenHeight = WINDOW_HEIGHT
  ) {
    // Check if the circle should bounce off of the screen edge
    bool circleIsOutOfBoundsX =
      position.x >= (screenWidth - radius) || position.x <= radius;
    bool circleIsOutOfBoundsY =
      position.y >= (screenHeight - radius) || position.y <= radius;
    if (circleIsOutOfBoundsX) {
      velocity.x *= -1.0f;
    }
    if (circleIsOutOfBoundsY) {
      velocity.y *= -1.0f;
    }
  }

  void setPosition(const Vector2 newPosition) {
    position = newPosition;
    refreshGridPositions();
  }

  void refreshGridPositions() {
    // Get the min(bottom-left) and max(top-right) extents of the Circle (just
    // like an AABB)
    Vector2 min = {position.x - radius, position.y + radius};
    Vector2 max = {position.x + radius, position.y - radius};

    Vector2 minGridPosition = convertToGridPosition(min);
    Vector2 maxGridPosition = convertToGridPosition(max);

    // It also occupies spaces that are in between min and max
    for (int i = minGridPosition.x; i < maxGridPosition.x; i++) {
      for (int j = minGridPosition.y; j > maxGridPosition.y; j--) {
      }
    }
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

  static Vector2 convertToGridPosition(const Vector2 position) {
    Vector2 gridPosition = {
      floor(position.x / GRID_SIZE), floor(position.y / GRID_SIZE)};
    return gridPosition;
  }

  // If a and b share at least one gridPosition, return true
  // static bool sharesGridPositions(const Circle& a, const Circle& b) {
  //   for (size_t i = 0; i < a.gridPositions.size(); i++) {
  //     for (size_t j = 0; j < b.gridPositions.size(); j++) {
  //       if (a.gridPositions[i].x == b.gridPositions[j].x
  // 			&& a.gridPositions[i].y == b.gridPositions[j].y)
  //         return true;
  //     }
  //   }
  //   return false;
  // }
};

struct Cell {
  Vector2 topRight;
  int size = GRID_SIZE;
  std::vector<Circle> objects;

  Cell() {}

  Cell(const Vector2 newTopRight) { topRight = newTopRight; }

  void draw() {
    DrawRectangleLines(topRight.x, topRight.y, GRID_SIZE, GRID_SIZE, RED);
  }

  // Take note of objects inside cell
  void refreshObjects() {}
};

struct UniformGrid {
  std::vector<std::vector<Cell>> cells;  // [row][column]

  UniformGrid() {
    for (size_t i = 0; i < WINDOW_HEIGHT; i += GRID_SIZE) {
      std::vector<Cell> row;
      for (size_t j = 0; j < WINDOW_WIDTH; j += GRID_SIZE) {
        Vector2 cellTopRight = {static_cast<float>(j), static_cast<float>(i)};
        row.push_back(Cell(cellTopRight));
      }
      cells.push_back(row);
    }
  }

  void draw() {
    for (size_t i = 0; i < cells.size(); i++) {
      for (size_t j = 0; j < cells[i].size(); j++) {
				cells[i][j].draw();
      }
    }
  }
};

int main() {
  srand(GetTime());

  // Counts the number of times the user has spawned a batch of small circles
  int numberOfSpawnKeyPresses = 0;

  UniformGrid uniformGrid = UniformGrid();

  std::vector<Circle> smallCircles;
  std::vector<Circle> bigCircles;

  char smallCircleCountBuffer[50];
  int numberOfSmallCirclesPresentFormatted;
  char bigCircleCountBuffer[50];
  int numberOfBigCirclesPresentFormatted;

  float accumulator(0.0f);
  float deltaTime(0.0f);

  bool paused(false);
  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME);
  SetTargetFPS(TARGET_FPS);
  while (!WindowShouldClose()) {
    deltaTime = GetFrameTime();

    if (IsKeyPressed(KEY_A)) {
      paused = !paused;
    }

    if (!paused) {
      if (IsKeyPressed(SPAWN_KEY)) {
        numberOfSpawnKeyPresses += 1;
        // If user reaches 10 presses, spawn a big boy
        if (numberOfSpawnKeyPresses % 10 == 0) {
          bigCircles.push_back(Circle());
          bigCircles[bigCircles.size() - 1].spawn(CircleSize::big);
          numberOfSpawnKeyPresses = 0;
        }

        // Spawn small circles
        int numberOfSmallCirclesAfterSpawning =
          smallCircles.size() + SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY;
        for (size_t i = smallCircles.size();
             i < numberOfSmallCirclesAfterSpawning; i++) {
          smallCircles.push_back(Circle());
          smallCircles[i].spawn();
        }
      }

      // Physics update
      accumulator += deltaTime;
      while (accumulator >= TIMESTEP) {
        for (size_t i = 0; i < smallCircles.size(); i++) {
          Circle* currentCircle = &smallCircles[i];
          currentCircle->handleCircleCollision(smallCircles, i);
          currentCircle->handleCircleCollision(bigCircles);
          currentCircle->handleEdgeCollision();
          currentCircle->update();
        }

        for (size_t i = 0; i < bigCircles.size(); i++) {
          Circle* currentCircle = &bigCircles[i];
          currentCircle->handleCircleCollision(smallCircles);
          currentCircle->handleCircleCollision(bigCircles, i);
          currentCircle->handleEdgeCollision();
          currentCircle->update();
        }
        accumulator -= TIMESTEP;
      }
    }

    // Draw
    BeginDrawing();
    ClearBackground(WHITE);

    // Draw grid
    uniformGrid.draw();

    // Draw circle
    for (size_t i = 0; i < smallCircles.size(); i++) {
      smallCircles[i].draw();
    }
    for (size_t i = 0; i < bigCircles.size(); i++) {
      bigCircles[i].draw();
    }

    // Small Circle Counter
    numberOfSmallCirclesPresentFormatted =
      sprintf(smallCircleCountBuffer, "%d Small Circles", smallCircles.size());
    DrawText(smallCircleCountBuffer, 10, 10, 20, BLACK);
    // Big Circle Counter
    numberOfBigCirclesPresentFormatted =
      sprintf(bigCircleCountBuffer, "%d Big Circles", bigCircles.size());
    DrawText(bigCircleCountBuffer, 10, 30, 20, BLACK);

    EndDrawing();
  }

  return 0;
}