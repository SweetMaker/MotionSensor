#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include "Quaternion_16384.h"
#include <iostream>

using namespace SweetMaker;
using namespace std;

typedef void (*TestFunction)(void);

void expectIntEquals(int32_t expected, int32_t received) {
	if (expected != received) {
		printf("expectIntEquals failed\n");
		printf("expected %d\n", expected);
		printf("received %d\n", received);
		throw std::runtime_error("expectIntEquals failed\n");
	}
}

void expectIntCloseTo(int32_t expected, int32_t received, int32_t delta) {
	if ((received < (expected - delta)) || (received > (expected + delta))) {
		printf("expectIntCloseTo failed\n");
		printf("expected %d, delta %d\n", expected, delta);
		printf("received %d\n", received);
		throw std::runtime_error("expectIntCloseTo failed\n");
	}
}


void expectQuatEquals(Quaternion_16384* expected, int16_t r, int16_t x, int16_t y, int16_t z) {
	if ((expected->r != r) || (expected->x != x) || (expected->y != y) || (expected->z != z)) {
		printf("expectQuatEquals failed\n");
		printf("expected %d:%d:%d:%d\n", expected->r, expected->x, expected->y, expected->z);
		printf("received %d:%d:%d:%d\n", r, x, y, z);
		throw std::runtime_error("expectQuatEquals failed\n");
	}
}

void expectQuatEquals(int16_t r, int16_t x, int16_t y, int16_t z, Quaternion_16384* received ) {
	if ((received->r != r) || (received->x != x) || (received->y != y) || (received->z != z)) {
		printf("expectQuatEquals failed\n");
		printf("expected %d:%d:%d:%d\n", r, x, y, z);
		printf("received %d:%d:%d:%d\n", received->r, received->x, received->y, received->z);
		throw std::runtime_error("expectQuatEquals failed\n");
	}
}


void expectQuatEquals(Quaternion_16384* expected, Quaternion_16384* received) {
	expectQuatEquals(expected, received->r, received->x, received->y, received->z);
}


void expectQuatCloseTo(Quaternion_16384* expected, Quaternion_16384* received, uint16_t delta) {
	try {
		expectIntCloseTo(expected->r, received->r, delta);
		expectIntCloseTo(expected->x, received->x, delta);
		expectIntCloseTo(expected->y, received->y, delta);
		expectIntCloseTo(expected->z, received->z, delta);
	}
	catch (std::runtime_error ex) {
		printf("expectQuatCloseTo failed\n");
		printf("expected %d:%d:%d:%d\n", expected->r, expected->x, expected->y, expected->z);
		printf(" with delta %d\n", delta);
		printf("received %d:%d:%d:%d\n", received->r, received->x, received->y, received->z);
		throw std::runtime_error("expectQuatEquals failed\n");
	}
}

void expectQuatCloseTo(int16_t r, int16_t x, int16_t y, int16_t z, Quaternion_16384* received, uint16_t delta) {
	try {
		expectIntCloseTo(r, received->r, delta);
		expectIntCloseTo(x, received->x, delta);
		expectIntCloseTo(y, received->y, delta);
		expectIntCloseTo(z, received->z, delta);
	}
	catch (std::runtime_error ex) {
		printf("expectQuatCloseTo failed\n");
		printf("expected %d:%d:%d:%d\n", r, x, y, z);
		printf(" with delta %d\n", delta);
		printf("received %d:%d:%d:%d\n", received->r, received->x, received->y, received->z);
		throw std::runtime_error("expectQuatEquals failed\n");
	}
}


void asrRounded() {
	printf("%s\n", __FUNCTION__);
	expectIntEquals(-2, Quaternion_16384::asrRounded(-25, 4));
	expectIntEquals(-1, Quaternion_16384::asrRounded(-24, 4));
	expectIntEquals(-1, Quaternion_16384::asrRounded(-16, 4));
	expectIntEquals(-1, Quaternion_16384::asrRounded(-9, 4));
	expectIntEquals(0, Quaternion_16384::asrRounded(-8, 4));
	expectIntEquals(0, Quaternion_16384::asrRounded(0, 4));
	expectIntEquals(0, Quaternion_16384::asrRounded(8, 4));
	expectIntEquals(1, Quaternion_16384::asrRounded(9, 4));
	expectIntEquals(1, Quaternion_16384::asrRounded(15, 4));
	expectIntEquals(1, Quaternion_16384::asrRounded(16, 4));
	expectIntEquals(1, Quaternion_16384::asrRounded(24, 4));
	expectIntEquals(2, Quaternion_16384::asrRounded(25, 4));
	expectIntEquals(2, Quaternion_16384::asrRounded(31, 4));
	expectIntEquals(2, Quaternion_16384::asrRounded(32, 4));
	expectIntEquals(2, Quaternion_16384::asrRounded(40, 4));
	expectIntEquals(3, Quaternion_16384::asrRounded(41, 4));
}

void crossProduct() {
	printf("%s\n", __FUNCTION__);
	Quaternion_16384 a(0, 16384, 0, 0);
	Quaternion_16384 b(0, 0, 16384, 0);
	a.crossProduct(&b);
	expectQuatEquals(0, 0, 0, 16384, &a);

	Quaternion_16384 c(0, 0, 16384, 0);
	Quaternion_16384 d(0, 0, 0, 16384);
	c.crossProduct(&d);
	expectQuatEquals(0, 16384, 0, 0, &c);

	Quaternion_16384 e(0, 0, 0, 16384);
	Quaternion_16384 f(0, 16384, 0, 0);
	e.crossProduct(&f);
	expectQuatEquals(0, 0, 16384, 0, &e);
}

void createRotationQuaternion() {
	printf("%s\n", __FUNCTION__);
	RotationQuaternion_16384 rotate90AboutZ((float)90, 0, 0, 16384);
	expectQuatEquals(11585, 0, 0, 11585, &rotate90AboutZ);
	
	RotationQuaternion_16384 rotate45AboutZ((float)45, 0, 0, 16384);
	expectQuatEquals(15137, 0, 0, 6270, &rotate45AboutZ);
}

void rotateQuaternionAboutX() {
	printf("%s\n", __FUNCTION__);
	Quaternion_16384 z_axis(0, 0, 0, 1000);
	RotationQuaternion_16384 rotate90AboutX((float)-90, 16384, 0, 0);

	rotate90AboutX.rotate(&z_axis);
	expectQuatEquals(0, 0, 1000, 0, &z_axis); // pointing at y

	rotate90AboutX.rotate(&z_axis);
	expectQuatEquals(0, 0, 0, -1000, &z_axis); // pointing at -z

	rotate90AboutX.rotate(&z_axis);
	expectQuatEquals(0, 0, -1000, 0, &z_axis); // pointing at -y

	rotate90AboutX.rotate(&z_axis);
	expectQuatEquals(0, 0, 0, 1000, &z_axis); // and back to z
}

void rotateQuaternionAboutZ() {
	printf("%s\n",__FUNCTION__);
	Quaternion_16384 x_axis(0, 1000, 0, 0);
	RotationQuaternion_16384 rotate90AboutZ((float)90, 0, 0, 16384);

	rotate90AboutZ.rotate(&x_axis);
	expectQuatEquals(0, 0, 1000, 0, &x_axis); // pointing at y
	
	rotate90AboutZ.rotate(&x_axis);
	expectQuatEquals(0, -1000, 0, 0, &x_axis); // pointing at -x

	rotate90AboutZ.rotate(&x_axis);
	expectQuatEquals(0, 0, -1000, 0, &x_axis); // pointing at -y

	rotate90AboutZ.rotate(&x_axis);
	expectQuatEquals(0, 1000, 0, 0, &x_axis); // and back to x
}

void createOffset() {
	printf("%s\n", __FUNCTION__);
	Quaternion_16384 a(0, 123, -234, 56);
	a.normalize();
	Quaternion_16384 b(0, 56, 743, -325);
	b.normalize();

	RotationQuaternion_16384 offsetRotation;

	offsetRotation.findOffsetRotation(&a, &b);
	offsetRotation.rotate(&a);
	expectQuatCloseTo(&a, &b, 10);
}


void getGravity() {
	printf("%s\n", __FUNCTION__);
	RotationQuaternion_16384 nullRotation((float)0, 0, 0, 16384);
	RotationQuaternion_16384 rotateAboutX((float)30, 16384, 0, 0);
	RotationQuaternion_16384 rotateAboutY((float)30, 0, 16384, 0);
	RotationQuaternion_16384 rotateAboutZ((float)30, 0, 0, 16384);
	RotationQuaternion_16384 rotateAboutXY((float)45, 100, 100, 0);

	Quaternion_16384 gravity;
	nullRotation.getGravity(&gravity);
	expectQuatEquals((float)0, 0, 0, 16384, &gravity);

	rotateAboutX.getGravity(&gravity);
	expectQuatCloseTo(0, 0, 8191, 14190, &gravity, 1);

	rotateAboutY.getGravity(&gravity);
	expectQuatCloseTo(0, -8191, 0, 14190, &gravity, 1);

	RotationQuaternion_16384 rotateZX;
	rotateZX = Quaternion_16384::crossProduct(&rotateAboutZ, &rotateAboutX);
	rotateZX.getGravity(&gravity);
	expectQuatCloseTo(0, 0, 8192, 14190, &gravity, 1);
}

void createRotationOffset1() {
	printf("%s\n", __FUNCTION__);
	RotationQuaternion_16384 rotateAboutX((float)30, 16384, 0, 0);
	RotationQuaternion_16384 rotateAboutZ((float)45, 0, 0, 16384);
	RotationQuaternion_16384 rotateZX;
	rotateZX = Quaternion_16384::crossProduct(&rotateAboutZ, &rotateAboutX);

	RotationQuaternion_16384 rot_z = rotateZX.getRotationAboutZ();
	rot_z.conjugate();
	RotationQuaternion_16384 offsetRot;
	offsetRot = Quaternion_16384::crossProduct(&rot_z, &rotateZX);
	expectQuatCloseTo(&rotateAboutX, &offsetRot, 2);
}

void createRotationOffset2() {
	printf("%s\n", __FUNCTION__);
	RotationQuaternion_16384 rot_z, offsetRot, rotateZXY;

	RotationQuaternion_16384 xy_rotation((float)30, 100, 90, 0);
	RotationQuaternion_16384 z_rotation((float)90, 0, 0, 16384);

	rotateZXY = Quaternion_16384::crossProduct(&z_rotation, &xy_rotation);
	rot_z = rotateZXY.getRotationAboutZ();
	rot_z.conjugate();

	offsetRot = Quaternion_16384::crossProduct(&rot_z, &rotateZXY);
	expectQuatCloseTo(&xy_rotation, &offsetRot, 2);
}


TestFunction testFunctions[] = {
	asrRounded,
	crossProduct,
	createRotationQuaternion,
	rotateQuaternionAboutX,
	rotateQuaternionAboutZ,
	getGravity,
	createOffset,
	createRotationOffset1,
	createRotationOffset2
};


int main() {
	printf("Motion Sensor Library Testing\n");

	for (TestFunction testFunc : testFunctions)	{
		try {
			printf("Starting new test: ");
			testFunc();
		}
		catch(std::runtime_error ex){
			printf("Test Failed\n");
			std::cout << ex.what();
		}
	}

	printf("Finished - press any key to exit\n");
	getchar();
	return (0);
}
