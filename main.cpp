#include <string.h>
#include <stdio.h>
#include <raylib.h>
#include <raymath.h>


const int WINDOW_WIDTH(1280);
const int WINDOW_HEIGHT(720);
const char* WINDOW_NAME("Spatial Data Structures");

const int TARGET_FPS(60);
const float TIMESTEP(1.0f / TARGET_FPS);

const KeyboardKey SPAWN_KEY(KEY_SPACE);

const int SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY(25);
const int SMALL_CIRCLE_RADIUS_MIN(5);
const int SMALL_CIRCLE_RADIUS_MAX(10);
const int SMALL_CIRCLE_MASS(1);
const float SMALL_CIRCLE_VELOCITY_MIN(50.0f);
const float SMALL_CIRCLE_VELOCITY_MAX(100.0f);

enum CircleSize {
	small = 0,
	big = 1
};

const int NUMBER_OF_PRESSES_UNTIL_BIG_CIRCLE_SPAWNS(10);
const int BIG_CIRCLE_RADIUS(25);
const int BIG_CIRCLE_MASS(10);

const float FRICTION(-0.75f);
const float VELOCITY_THRESHOLD(5.0f);
const float ELASTICITY(0.5f);

static float randf(const float min, const float max);

struct Circle {
  int radius;
  int mass;
  Color color;

  Vector2 acceleration;
  Vector2 velocity;
  Vector2 position;

	// If big, spawn at bottom middle of screen
	// Else, spawn at middle
  void spawn(const CircleSize size = small) { 
		color = {
			static_cast<unsigned char>(rand() % 256),
			static_cast<unsigned char>(rand() % 256),
			static_cast<unsigned char>(rand() % 256),
			255
		};
		velocity = {
			randf(SMALL_CIRCLE_VELOCITY_MIN, SMALL_CIRCLE_VELOCITY_MAX) * randf(-1.0f, 1.0f),
			randf(SMALL_CIRCLE_VELOCITY_MIN, SMALL_CIRCLE_VELOCITY_MAX) * randf(-1.0f, 1.0f)
		};
		position = {
			WINDOW_WIDTH / 2,
			WINDOW_HEIGHT / 2
		};
		radius = rand() % (SMALL_CIRCLE_RADIUS_MAX - SMALL_CIRCLE_RADIUS_MIN) + SMALL_CIRCLE_RADIUS_MIN;
	}

  void draw() { DrawCircle(position.x, position.y, radius, color); }

	void update(Vector2 force = {0.0f, 0.0f}, float timestep = TIMESTEP) {
    // acceleration = Vector2Add(
    //   Vector2Scale(force, 1 / mass), (Vector2Scale(velocity, FRICTION))
    // ); // With friction
		acceleration = Vector2Scale(force, 1 / mass); // No friction
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

// https://cplusplus.com/forum/beginner/81180/
// Returns a random float within min and max
static float randf(const float min, const float max) {
  float result = (rand() / static_cast<float>(RAND_MAX) * (max + 1)) + min;
  return result;
}

// https://stackoverflow.com/questions/3749660/how-to-resize-array-in-c
static void doubleCircleArraySize(Circle* arr, size_t& size) {
  const size_t newSize = size * 2;
  Circle* newArr = new Circle[newSize];

  memcpy(newArr, arr, size * sizeof(Circle));

  size = newSize;
  delete[] arr;
  arr = newArr;
}

int main() {
  size_t smallCirclesArraySize = SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY;
  int numberOfSmallCirclesPresent = 0;
	int numberOfSpawnKeyPresses = 0; // Counts the number of times the user has spawned 10 small circles
  Circle* smallCircles = new Circle[smallCirclesArraySize];

	float accumulator(0.0f);
	float deltaTime(0.0f);

  InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_NAME);
  SetTargetFPS(TARGET_FPS);
  while (!WindowShouldClose()) {
		deltaTime = GetFrameTime();


		if (IsKeyPressed(SPAWN_KEY)) {
			// Try to spawn 10 small circles
			int numberOfSmallCirclesAfterSpawning = numberOfSmallCirclesPresent + SMALL_CIRCLES_TO_SPAWN_SIMULTANEOUSLY;
			// If there is no more space in the array, double its size
			if (numberOfSmallCirclesAfterSpawning > smallCirclesArraySize) {
				doubleCircleArraySize(smallCircles, smallCirclesArraySize);
				printf("%d\n", smallCirclesArraySize);
			}
			// Spawn 10 small circles
			for (size_t i = numberOfSmallCirclesPresent; i < numberOfSmallCirclesAfterSpawning; i++) {
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
						currentCircle->position.x >= (WINDOW_WIDTH - currentCircle->radius)
						|| currentCircle->position.x <= currentCircle->radius;
				bool circleIsOutOfBoundsY =
						currentCircle->position.y >= (WINDOW_HEIGHT - currentCircle->radius)
						|| currentCircle->position.y <= currentCircle->radius;
				if (circleIsOutOfBoundsX) { currentCircle->velocity.x *= -1.0f; }
				if (circleIsOutOfBoundsY) { currentCircle->velocity.y *= -1.0f; }

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

		EndDrawing();
  }

	delete[] smallCircles;
  return 0;
}