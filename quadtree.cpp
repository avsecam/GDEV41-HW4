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
const KeyboardKey PAUSE_KEY(KEY_A);
const KeyboardKey DETAILS_KEY(KEY_Q);

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

const float FRICTION(-0.75f);
const float VELOCITY_THRESHOLD(5.0f);
const float ELASTICITY(0.5f);

enum QuadPosition { none = -1, topLeft = 0, topRight = 1, bottomLeft = 2, bottomRight = 3 };

const int MAX_DEPTH(5);

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
    position = Vector2Add(position, Vector2Scale(velocity, TIMESTEP));
  }

  void handleCircleCollision(
    const std::vector<Circle*> circles, const size_t idx = -1
  ) {
    // Choose if the loop should start at 0 or at idx
    size_t iterator = (idx == -1) ? 0 : idx;
    for (size_t i = iterator; i < circles.size(); i++) {
      Circle* a = this;
      Circle b = *circles[i];

      // if (a == &b) continue;

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
  Vector2 topLeft;
  int size;
  int depth;

  Quad* parent;

  Quad* topLeftChild;
  Quad* topRightChild;
  Quad* bottomLeftChild;
  Quad* bottomRightChild;

  std::vector<Circle*> objects;

  Quad() {
    topLeft = {0, 0};
    size = (WINDOW_HEIGHT > WINDOW_WIDTH) ? WINDOW_HEIGHT : WINDOW_WIDTH;
    depth = 1;
  }

  Quad(const Vector2 _topLeft, const int _size, const int _depth) {
    topLeft = _topLeft;
    size = _size;
    depth = (_depth > MAX_DEPTH) ? MAX_DEPTH : _depth;  // Limit the depth
  }

  // Show quad and number of objects inside
  void draw(const int x = -1, const int y = -1) {
    DrawRectangleLines(topLeft.x, topLeft.y, size, size, RED);
    if (x >= 0 && y >= 0) {
      char buffer[10];

      sprintf(buffer, "%d", objects.size());
      DrawText(
        buffer, topLeft.x + (size / 2), topLeft.y + (size / 2), 15, GREEN
      );
    }
  }

  // Subdivide quad
  Quad* makeChildQuad(QuadPosition position) {
    float halfSize = size / 2;
    switch (position) {
      case QuadPosition::topLeft:
        topLeftChild = new Quad(topLeft, halfSize, depth + 1);
				return topLeftChild;
        break;

      case QuadPosition::topRight:
        topRightChild =
          new Quad({topLeft.x + halfSize, topLeft.y}, halfSize, depth + 1);
				return topRightChild;
        break;

      case QuadPosition::bottomLeft:
        bottomLeftChild =
          new Quad({topLeft.x, topLeft.y + halfSize}, halfSize, depth + 1);
				return bottomLeftChild;
        break;

      case QuadPosition::bottomRight:
        bottomRightChild =
          new Quad(Vector2AddValue(topLeft, halfSize), halfSize, depth + 1);
				return bottomRightChild;
        break;

      default:
        break;
    }
  }

  // Return whether the quad can COMPLETELY contain the circle's AABB
  bool canContainCircle(const Circle* circle) {
    Vector2 quadTopLeft = topLeft;
    Vector2 quadBottomRight = Vector2AddValue(topLeft, size);

    Vector2 circleTopLeft =
      Vector2SubtractValue(circle->position, circle->radius);
    Vector2 circleBottomRight =
      Vector2AddValue(circle->position, circle->radius);

    return (
      circleTopLeft.x >= quadTopLeft.x && circleTopLeft.y >= quadTopLeft.y &&
      circleBottomRight.x <= quadBottomRight.x &&
      circleBottomRight.y >= quadBottomRight.y
    );
  }

	// Check if the circle can be contained in the quad's children
	// If it can, create a quad that contains the circle
	QuadPosition childCanContainCircle(const Circle* circle) {
    float halfSize = size / 2;
		Quad* tempQuad;

		// Top left
		tempQuad = new Quad(topLeft, halfSize, depth + 1);
		if (tempQuad->canContainCircle(circle)) return QuadPosition::topLeft;

		// Top right
		tempQuad->topLeft = { tempQuad->topLeft.x + halfSize, tempQuad->topLeft.y };
		if (tempQuad->canContainCircle(circle)) return QuadPosition::topRight;
		
		// Bottom left
		tempQuad->topLeft = { tempQuad->topLeft.x - halfSize, tempQuad->topLeft.y + halfSize};
		if (tempQuad->canContainCircle(circle)) return QuadPosition::bottomLeft;
		
		// Bottom right
		tempQuad->topLeft = { tempQuad->topLeft.x + halfSize, tempQuad->topLeft.y };
		if (tempQuad->canContainCircle(circle)) return QuadPosition::bottomRight;

		return QuadPosition::none;
	}

  // Insert an object into the appropriate quad
  void insert(Circle* circle) {
		// Leaf check
		if (depth >= MAX_DEPTH) {
			objects.push_back(circle);
			return;
		}
		
		// Check child nodes if one of them can contain the circle completely
		QuadPosition childPositionThatContainsCircle = childCanContainCircle(circle);

		// If no child can completely contain the circle
		if (childPositionThatContainsCircle == QuadPosition::none) {
			objects.push_back(circle);
			return;
		} else {
			Quad* childThatContainsCircle = makeChildQuad(childPositionThatContainsCircle);
			childThatContainsCircle->insert(circle);
		}
	}

	// Recursively clear all quads of objects
	void clear() {
		if (!topLeftChild && !topRightChild && !bottomLeftChild && !bottomRightChild) objects.clear();

		if (topLeftChild) topLeftChild->clear();
		if (topRightChild) topRightChild->clear();
		if (bottomLeftChild) bottomLeftChild->clear();
		if (bottomRightChild) bottomRightChild->clear();
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
        for (size_t i = 0; i < circles.size(); i++) {
          circles[i]->update();
        }


        accumulator -= TIMESTEP;
      }
    }

    // Draw
    BeginDrawing();
    ClearBackground(WHITE);

    if (showTree) {

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

    EndDrawing();
  }

  return 0;
}