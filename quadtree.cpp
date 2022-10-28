#include <raylib.h>
#include <raymath.h>
#include <stdio.h>
#include <string.h>

#include <vector>

const int WINDOW_WIDTH(1280);
const int WINDOW_HEIGHT(720);
const char* WINDOW_NAME("Spatial Data Structures - Quadtree");

const int TARGET_FPS(60);
const float TIMESTEP(1.0f / TARGET_FPS);

const KeyboardKey SPAWN_KEY(KEY_SPACE);
const KeyboardKey PAUSE_KEY(KEY_A);
const KeyboardKey DETAILS_KEY(KEY_Q);

enum CircleSize { small = 0, big = 1 };

const float CIRCLE_VELOCITY_MIN(5.0f);
const float CIRCLE_VELOCITY_MAX(300.0f);

const int SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY(25);
const int SMALL_CIRCLE_RADIUS_MIN(5);
const int SMALL_CIRCLE_RADIUS_MAX(10);
const int SMALL_CIRCLE_MASS(1);

const int NUMBER_OF_PRESSES_UNTIL_BIG_CIRCLE_SPAWNS(10);
const int BIG_CIRCLE_RADIUS(25);
const int BIG_CIRCLE_MASS(10);

const float FRICTION(-0.75f);
const float VELOCITY_THRESHOLD(5.0f);
const float ELASTICITY(1.0f);

enum QuadPosition {
  none = -1,
  topLeft = 0,
  topRight = 1,
  bottomLeft = 2,
  bottomRight = 3
};

const int MAX_DEPTH(4);

struct Circle;
struct Quad;

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
  Vector2 oldPosition;

  Quad* quad = nullptr;

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
      position = {WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2};
      velocity.y =
        randf(CIRCLE_VELOCITY_MIN, CIRCLE_VELOCITY_MAX) * directionMultiplier();
    } else {
      radius = BIG_CIRCLE_RADIUS;
      mass = BIG_CIRCLE_MASS;
      position = {WINDOW_WIDTH / 2, WINDOW_HEIGHT - (float)radius};
      velocity.y = randf(CIRCLE_VELOCITY_MIN, CIRCLE_VELOCITY_MAX);
    }
  }

  void draw() { DrawCircle(position.x, position.y, radius, color); }

  void update(
    const Vector2 force = {0.0f, 0.0f}, const float timestep = TIMESTEP
  ) {
    // acceleration = Vector2Add(
    //   Vector2Scale(force, 1 / mass), (Vector2Scale(velocity, FRICTION))
    // ); // With friction
    acceleration = Vector2Scale(force, 1 / mass);  // No friction
    velocity = Vector2Add(velocity, Vector2Scale(acceleration, TIMESTEP));
    velocity.x = (abs(velocity.x) < VELOCITY_THRESHOLD) ? 0.0f : velocity.x;
    velocity.y = (abs(velocity.y) < VELOCITY_THRESHOLD) ? 0.0f : velocity.y;
    oldPosition = position;
    position = Vector2Add(position, Vector2Scale(velocity, TIMESTEP));

    quad = nullptr;
  }

  void handleCircleCollision(const std::vector<Circle*> circles) {
    for (size_t i = 0; i < circles.size(); i++) {
      Circle* a = this;
      Circle* b = circles[i];

      if (a == b) continue;

      float sumOfRadii(pow(a->radius + b->radius, 2));
      float distanceBetweenCenters(Vector2DistanceSqr(a->position, b->position)
      );

      // Collision detected
      if (sumOfRadii >= distanceBetweenCenters) {
        Vector2 collisionNormalAB(
          {b->position.x - a->position.x, b->position.y - a->position.y}
        );
        Vector2 relativeVelocityAB(Vector2Subtract(a->velocity, b->velocity));
        Vector2 collisionNormalABNormalized(Vector2Normalize(collisionNormalAB)
        );
        Vector2 relativeVelocityABNormalized(Vector2Normalize(relativeVelocityAB
        ));

        // Collision response
        // Check dot product between collision normal and relative velocity
        if (Vector2DotProduct(relativeVelocityABNormalized, collisionNormalABNormalized) > 0) {
          float impulse =
            Circle::getImpulse(*a, *b, relativeVelocityAB, collisionNormalAB);
          a->velocity = Vector2Add(
            a->velocity,
            Vector2Scale(
              Vector2Scale(collisionNormalAB, 1.0f / a->mass), impulse
            )
          );
          b->velocity = Vector2Subtract(
            b->velocity,
            Vector2Scale(
              Vector2Scale(collisionNormalAB, 1.0f / b->mass), impulse
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
      position = oldPosition;
      velocity.x *= -1.0f;
    }
    if (circleIsOutOfBoundsY) {
      position = oldPosition;
      velocity.y *= -1.0f;
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
};

// https://www.geeksforgeeks.org/quad-tree/
struct Quad {
  Vector2 center;
  int halfWidth;
  int depth;

  Quad* parent;  // Not used as of the moment

  Quad* topLeftChild = nullptr;
  Quad* topRightChild = nullptr;
  Quad* bottomLeftChild = nullptr;
  Quad* bottomRightChild = nullptr;

  std::vector<Circle*> objects;

  Quad() {
    center = {WINDOW_WIDTH / 2, WINDOW_HEIGHT / 2};
    halfWidth =
      (WINDOW_HEIGHT > WINDOW_WIDTH) ? WINDOW_HEIGHT / 2 : WINDOW_WIDTH / 2;
    depth = 1;

    subdivide();
  }

  Quad(const Vector2 _center, const int _halfWidth, const int _depth) {
    center = _center;
    halfWidth = _halfWidth;
    depth = (_depth > MAX_DEPTH) ? MAX_DEPTH : _depth;  // Limit the depth

    if (depth < MAX_DEPTH) subdivide();
  }

  // Show quad and number of objects inside
  void draw() {
    if (branchContainsObjects()) {
      Vector2 topLeft = Vector2SubtractValue(center, halfWidth);
      DrawRectangleLines(
        topLeft.x, topLeft.y, halfWidth * 2, halfWidth * 2, RED
      );
    }

    if (topLeftChild) topLeftChild->draw();
    if (topRightChild) topRightChild->draw();
    if (bottomLeftChild) bottomLeftChild->draw();
    if (bottomRightChild) bottomRightChild->draw();
  }

  // Return true if any of the children, grandchildren, etc. contains at least
  // one circle
  bool branchContainsObjects() {
    if (depth >= MAX_DEPTH) return !objects.empty();

    if (!objects.empty()) return true;

    return topLeftChild->branchContainsObjects() ||
           topRightChild->branchContainsObjects() ||
           bottomLeftChild->branchContainsObjects() ||
           bottomRightChild->branchContainsObjects();
  }

  // Return circles that are near this circle
  std::vector<Circle*> getObjectsForCollisionCheck(const Circle* circle) {
    std::vector<Circle*> circles;
		if (!isOverlapping(circle, this)) return circles;

    if (depth >= MAX_DEPTH) {
      for (size_t i = 0; i < objects.size(); i++) {
        circles.push_back(objects[i]);
      }
      return circles;
    }

    if (isOverlapping(circle, topLeftChild)) {
      std::vector<Circle*> topLeftChildCircles =
        topLeftChild->getObjectsForCollisionCheck(circle);
      for (size_t i = 0; i < topLeftChildCircles.size(); i++) {
        circles.push_back(topLeftChildCircles[i]);
      }
    }
    if (isOverlapping(circle, topRightChild)) {
      std::vector<Circle*> topRightChildCircles =
        topRightChild->getObjectsForCollisionCheck(circle);
      for (size_t i = 0; i < topRightChildCircles.size(); i++) {
        circles.push_back(topRightChildCircles[i]);
      }
    }
    if (isOverlapping(circle, bottomLeftChild)) {
      std::vector<Circle*> bottomLeftChildCircles =
        bottomLeftChild->getObjectsForCollisionCheck(circle);
      for (size_t i = 0; i < bottomLeftChildCircles.size(); i++) {
        circles.push_back(bottomLeftChildCircles[i]);
      }
    }
    if (isOverlapping(circle, bottomRightChild)) {
      std::vector<Circle*> bottomRightChildCircles =
        bottomRightChild->getObjectsForCollisionCheck(circle);
      for (size_t i = 0; i < bottomRightChildCircles.size(); i++) {
        circles.push_back(bottomRightChildCircles[i]);
      }
    }

    for (size_t i = 0; i < objects.size(); i++) {
      circles.push_back(objects[i]);
    }

    return circles;
  }

  // Subdivide quad
  void subdivide() {
    float halfOfHalfWidth = halfWidth / 2;
    topLeftChild = new Quad(
      {center.x - halfOfHalfWidth, center.y - halfOfHalfWidth}, halfOfHalfWidth,
      depth + 1
    );

    topRightChild = new Quad(
      {center.x + halfOfHalfWidth, center.y - halfOfHalfWidth}, halfOfHalfWidth,
      depth + 1
    );

    bottomLeftChild = new Quad(
      {center.x - halfOfHalfWidth, center.y + halfOfHalfWidth}, halfOfHalfWidth,
      depth + 1
    );

    bottomRightChild = new Quad(
      {center.x + halfOfHalfWidth, center.y + halfOfHalfWidth}, halfOfHalfWidth,
      depth + 1
    );
  }

  // Return whether the quad can COMPLETELY contain the circle's AABB
  bool canContainCircle(const Circle* circle) {
    Vector2 quadTopLeft = Vector2SubtractValue(center, halfWidth);
    Vector2 quadBottomRight = Vector2AddValue(center, halfWidth);

    Vector2 circleTopLeft =
      Vector2SubtractValue(circle->position, circle->radius);
    Vector2 circleBottomRight =
      Vector2AddValue(circle->position, circle->radius);

    return (
      circleTopLeft.x >= quadTopLeft.x && circleTopLeft.y >= quadTopLeft.y &&
      circleBottomRight.x <= quadBottomRight.x &&
      circleBottomRight.y <= quadBottomRight.y
    );
  }

  // Check if the circle can be contained in the quad's children
  // If it can, create a quad that contains the circle
  QuadPosition getPositionThatCanContainCircle(const Circle* circle) {
    QuadPosition position = QuadPosition::none;

    // Top left
    if (topLeftChild->canContainCircle(circle))
      position = QuadPosition::topLeft;

    // Top right
    if (topRightChild->canContainCircle(circle))
      position = QuadPosition::topRight;

    // Bottom left
    if (bottomLeftChild->canContainCircle(circle))
      position = QuadPosition::bottomLeft;

    // Bottom right
    if (bottomRightChild->canContainCircle(circle))
      position = QuadPosition::bottomRight;

    return position;
  }

  // Insert an object into the appropriate quad
  void insert(Circle* circle) {
    // Leaf check
    if (depth >= MAX_DEPTH) {
      objects.push_back(circle);
      circle->quad = this;
      return;
    }

    // Check child nodes if one of them can contain the circle completely
    QuadPosition childPositionThatContainsCircle =
      getPositionThatCanContainCircle(circle);

    // If no child can completely contain the circle
    if (childPositionThatContainsCircle == QuadPosition::none) {
      objects.push_back(circle);
      circle->quad = this;
      return;
    } else {
      // Pick quad which contains circle
      switch (childPositionThatContainsCircle) {
        case QuadPosition::topLeft:
          topLeftChild->insert(circle);
          break;

        case QuadPosition::topRight:
          topRightChild->insert(circle);
          break;

        case QuadPosition::bottomLeft:
          bottomLeftChild->insert(circle);
          break;

        case QuadPosition::bottomRight:
          bottomRightChild->insert(circle);
          break;

        default:
          break;
      }
    }
  }

  // Recursively free all quads of objects
  void clear() {
    objects.clear();
    if (depth >= MAX_DEPTH) {
      return;
    }

    if (topLeftChild) {
      topLeftChild->clear();
    }
    if (topRightChild) {
      topRightChild->clear();
    }
    if (bottomLeftChild) {
      bottomLeftChild->clear();
    }
    if (bottomRightChild) {
      bottomRightChild->clear();
    }
  }

  // Do physics recursively
  void update() {
    if (!objects.empty()) {
      std::vector<Circle*> objectsForCollisionCheck;
      for (size_t i = 0; i < objects.size(); i++) {
        // Check collision for objects that are in child quads of the circle's
        // current quad
        objectsForCollisionCheck =
          objects[i]->quad->getObjectsForCollisionCheck(objects[i]);
        objects[i]->handleCircleCollision(objectsForCollisionCheck);

        objects[i]->handleEdgeCollision();
      }
    }

    if (depth >= MAX_DEPTH) {
      return;
    }

    if (topLeftChild) topLeftChild->update();
    if (topRightChild) topRightChild->update();
    if (bottomLeftChild) bottomLeftChild->update();
    if (bottomRightChild) bottomRightChild->update();
  }
	
  // Return true if the circle's AABB and the quad are overlapping
  // https://developer.mozilla.org/en-US/docs/Games/Techniques/2D_collision_detection
  static bool isOverlapping(const Circle* c, const Quad* q) {
    Vector2 circleTopLeft = Vector2SubtractValue(c->position, c->radius);
    Vector2 circleBottomRight = Vector2AddValue(c->position, c->radius);

    Vector2 quadTopLeft = Vector2SubtractValue(q->center, q->halfWidth);
    Vector2 quadBottomRight = Vector2AddValue(q->center, q->halfWidth);

    return (
      circleTopLeft.x < quadTopLeft.x + quadBottomRight.x &&
      circleTopLeft.x + circleBottomRight.x > quadTopLeft.x &&
      circleTopLeft.y < quadTopLeft.y + quadBottomRight.y &&
      circleTopLeft.y + circleBottomRight.y > quadTopLeft.y
    );
  }
};

int main() {
  srand(GetTime());

  // Counts the number of times the user has spawned 10 small circles
  int numberOfSpawnKeyPresses = 0;

  Quad quadtree = Quad();

  std::vector<Circle*> circles;

  int numberOfSmallCirclesPresent = 0;
  int numberOfBigCirclesPresent = 0;

  char smallCircleCountBuffer[50];
  int numberOfSmallCirclesPresentFormatted;
  char bigCircleCountBuffer[50];
  int numberOfBigCirclesPresentFormatted;

  float accumulator(0.0f);
  float deltaTime(0.0f);

  bool paused(false);
  bool showTree(false);

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME);
  SetTargetFPS(TARGET_FPS);
  while (!WindowShouldClose()) {
    deltaTime = GetFrameTime();

    if (IsKeyPressed(PAUSE_KEY)) {
      paused = !paused;
    }

    if (IsKeyPressed(DETAILS_KEY)) {
      showTree = !showTree;
    }

    if (!paused) {
      if (IsKeyPressed(SPAWN_KEY)) {
        numberOfSpawnKeyPresses += 1;
        // If user reaches 10 presses, spawn a big boy
        if (numberOfSpawnKeyPresses % NUMBER_OF_PRESSES_UNTIL_BIG_CIRCLE_SPAWNS == 0) {
          circles.push_back(new Circle());
          circles[circles.size() - 1]->spawn(CircleSize::big);
          numberOfSpawnKeyPresses = 0;
          numberOfBigCirclesPresent += 1;
        }

        // Spawn small circles
        int numberOfSmallCirclesAfterSpawning =
          numberOfSmallCirclesPresent + SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY;
        for (size_t i = circles.size(); i < numberOfSmallCirclesAfterSpawning;
             i++) {
          circles.push_back(new Circle());
          circles[i]->spawn();
        }
        numberOfSmallCirclesPresent = numberOfSmallCirclesAfterSpawning;
      }

      // Physics update
      accumulator += deltaTime;
      while (accumulator >= TIMESTEP) {
        quadtree.clear();

        for (size_t i = 0; i < circles.size(); i++) {
          circles[i]->update();
          quadtree.insert(circles[i]);
        }

        quadtree.update();

        accumulator -= TIMESTEP;
      }
    }

    // Draw
    BeginDrawing();
    ClearBackground(WHITE);

    if (showTree) {
      quadtree.draw();
    }

    for (size_t i = 0; i < circles.size(); i++) {
      circles[i]->draw();
    }

    // Small Circle Counter
    numberOfSmallCirclesPresentFormatted = sprintf(
      smallCircleCountBuffer, "%d Small Circles", numberOfSmallCirclesPresent
    );
    DrawText(smallCircleCountBuffer, 10, 10, 20, BLACK);
    // Big Circle Counter
    numberOfBigCirclesPresentFormatted = sprintf(
      bigCircleCountBuffer, "%d Big Circles", numberOfBigCirclesPresent
    );
    DrawText(bigCircleCountBuffer, 10, 30, 20, BLACK);

    DrawText("Press Q to toggle uniform grid visibility.", 10, 50, 20, BLACK);

		if (paused) {
			DrawText("Press A to resume.", 150, (WINDOW_HEIGHT / 2) - 50, 100, ORANGE);
		} else {
			DrawText("Press A to pause.", 10, 70, 20, BLACK);
		}
    EndDrawing();
  }

  for (size_t i = 0; i < circles.size(); i++) {
    delete circles[i];
  }

  return 0;
}