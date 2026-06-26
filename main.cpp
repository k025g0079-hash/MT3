#include <Novice.h>
#include <math.h>
#include <assert.h>
#include <utility> 
#define ImGui_ImplDX12_RGB_Color
#include <imgui.h>

// --- 構造体定義 ---
struct Vector {
	float x;
	float y;
	float z;
};

struct Matrix4x4 {
	float m[4][4];
};

struct Plane {
	Vector normal;   // 平面の法線（必ず正規化されたベクトル）
	float distance;  // 原点から平面までの最短距離
};

struct Segment {
	Vector origin;   // 始点(A)
	Vector diff;     // 終点への差分ベクトル(B - A)
};

struct Triangle {
	Vector v0;       // 頂点0
	Vector v1;       // 頂点1
	Vector v2;       // 頂点2
};

// --- ベクトル演算 ---
Vector Add(Vector v1, Vector v2) { return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z }; }
Vector Subtract(Vector v1, Vector v2) { return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z }; }
Vector Multiply(float k, Vector v) { return { k * v.x, k * v.y, k * v.z }; }
float Dot(Vector v1, Vector v2) { return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z; }
float Length(Vector v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
Vector Normalize(Vector v) {
	float len = Length(v);
	if (len == 0.0f) return { 0, 0, 0 };
	return { v.x / len, v.y / len, v.z / len };
}
Vector Cross(Vector v1, Vector v2) {
	return {
		v1.y * v2.z - v1.z * v2.y,
		v1.z * v2.x - v1.x * v2.z,
		v1.x * v2.y - v1.y * v2.x
	};
}

// --- 行列演算 ---
Matrix4x4 Multiply(Matrix4x4 A, Matrix4x4 B) {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			result.m[i][j] = 0.0f;
			for (int k = 0; k < 4; k++) {
				result.m[i][j] += A.m[i][k] * B.m[k][j];
			}
		}
	}
	return result;
}

Matrix4x4 Inverse(Matrix4x4 m) {
	Matrix4x4 result{};
	float a[4][8] = {};
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			a[i][j] = m.m[i][j];
			a[i][j + 4] = (i == j) ? 1.0f : 0.0f;
		}
	}
	for (int i = 0; i < 4; i++) {
		int pivotRow = i;
		float maxVal = fabsf(a[i][i]);
		for (int k = i + 1; k < 4; k++) {
			if (fabsf(a[k][i]) > maxVal) {
				maxVal = fabsf(a[k][i]);
				pivotRow = k;
			}
		}
		if (pivotRow != i) {
			for (int j = 0; j < 8; j++) std::swap(a[i][j], a[pivotRow][j]);
		}
		float pivot = a[i][i];
		assert(fabsf(pivot) > 1e-6f);
		for (int j = 0; j < 8; j++) a[i][j] /= pivot;
		for (int k = 0; k < 4; k++) {
			if (k == i) continue;
			float factor = a[k][i];
			for (int j = 0; j < 8; j++) a[k][j] -= factor * a[i][j];
		}
	}
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) result.m[i][j] = a[i][j + 4];
	}
	return result;
}

Matrix4x4 MakeIdentity() {
	Matrix4x4 result{};
	for (int i = 0; i < 4; i++) result.m[i][i] = 1.0f;
	return result;
}

Matrix4x4 MakeAffineMatrix(Vector scale, Vector rotate, Vector translate) {
	Matrix4x4 rotateX = { {
		{1.0f, 0.0f, 0.0f, 0.0f},
		{0.0f, cosf(rotate.x), sinf(rotate.x), 0.0f},
		{0.0f, -sinf(rotate.x), cosf(rotate.x), 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotateY = { {
		{cosf(rotate.y), 0.0f, -sinf(rotate.y), 0.0f},
		{0.0f, 1.0f, 0.0f, 0.0f},
		{sinf(rotate.y), 0.0f, cosf(rotate.y), 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotateZ = { {
		{cosf(rotate.z), sinf(rotate.z), 0.0f, 0.0f},
		{-sinf(rotate.z), cosf(rotate.z), 0.0f, 0.0f},
		{0.0f, 0.0f, 1.0f, 0.0f},
		{0.0f, 0.0f, 0.0f, 1.0f}
	} };
	Matrix4x4 rotMat = Multiply(rotateX, Multiply(rotateY, rotateZ));

	Matrix4x4 result = { {
		{scale.x * rotMat.m[0][0], scale.x * rotMat.m[0][1], scale.x * rotMat.m[0][2], 0.0f},
		{scale.y * rotMat.m[1][0], scale.y * rotMat.m[1][1], scale.y * rotMat.m[1][2], 0.0f},
		{scale.z * rotMat.m[2][0], scale.z * rotMat.m[2][1], scale.z * rotMat.m[2][2], 0.0f},
		{translate.x, translate.y, translate.z, 1.0f}
	} };
	return result;
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspect, float nearClip, float farClip) {
	Matrix4x4 result{};
	float cot = 1.0f / tanf(fovY / 2.0f);
	result.m[0][0] = cot / aspect;
	result.m[1][1] = cot;
	result.m[2][2] = farClip / (farClip - nearClip);
	result.m[2][3] = 1.0f;
	result.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	return result;
}

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth) {
	Matrix4x4 result{};
	result.m[0][0] = width / 2.0f;
	result.m[1][1] = -height / 2.0f;
	result.m[2][2] = maxDepth - minDepth;
	result.m[3][0] = left + width / 2.0f;
	result.m[3][1] = top + height / 2.0f;
	result.m[3][2] = minDepth;
	result.m[3][3] = 1.0f;
	return result;
}

Vector Transform(Vector vector, Matrix4x4 matrix) {
	Vector result{};
	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + matrix.m[3][3];
	if (w == 0.0f) w = 1.0f;

	result.x = (vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + matrix.m[3][0]) / w;
	result.y = (vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + matrix.m[3][1]) / w;
	result.z = (vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + matrix.m[3][2]) / w;
	return result;
}

Vector Transform3DTo2D(Vector position, Matrix4x4 worldMatrix, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix) {
	Matrix4x4 wvpVpMatrix = Multiply(worldMatrix, Multiply(viewMatrix, Multiply(projectionMatrix, viewportMatrix)));
	return Transform(position, wvpVpMatrix);
}

// --- 衝突判定関数 ---

// 平面と線分
bool IsCollisionPlaneToSegment(Plane plane, Segment segment, float& outT, Vector& outPoint) {
	float denominator = Dot(plane.normal, segment.diff);

	if (fabsf(denominator) < 1e-6f) {
		return false;
	}

	float numerator = plane.distance - Dot(plane.normal, segment.origin);
	outT = numerator / denominator;

	if (outT >= 0.0f && outT <= 1.0f) {
		outPoint = Add(segment.origin, Multiply(outT, segment.diff));
		return true;
	}
	return false;
}

// 点が三角形の内側にあるか判定（内外判定用ヘルパー）
bool IsPointInsideTriangle(Vector p, Triangle triangle, Vector normal) {
	Vector v01 = Subtract(triangle.v1, triangle.v0);
	Vector v12 = Subtract(triangle.v2, triangle.v1);
	Vector v20 = Subtract(triangle.v0, triangle.v2);

	Vector v0p = Subtract(p, triangle.v0);
	Vector v1p = Subtract(p, triangle.v1);
	Vector v2p = Subtract(p, triangle.v2);

	Vector cross0 = Cross(v01, v0p);
	Vector cross1 = Cross(v12, v1p);
	Vector cross2 = Cross(v20, v2p);

	if (Dot(cross0, normal) >= 0.0f &&
		Dot(cross1, normal) >= 0.0f &&
		Dot(cross2, normal) >= 0.0f) {
		return true;
	}
	return false;
}

// 三角形と線分
bool IsCollisionTriangleToSegment(Triangle triangle, Segment segment, float& outT, Vector& outPoint) {
	Vector v01 = Subtract(triangle.v1, triangle.v0);
	Vector v02 = Subtract(triangle.v2, triangle.v0);
	Vector normal = Normalize(Cross(v01, v02));

	Plane plane;
	plane.normal = normal;
	plane.distance = Dot(normal, triangle.v0);

	if (IsCollisionPlaneToSegment(plane, segment, outT, outPoint)) {
		return IsPointInsideTriangle(outPoint, triangle, normal);
	}
	return false;
}

// --- 描画関数 ---

void DrawGrid(Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix) {
	const float kGridHalfWidth = 2.0f;
	const int kSubdivision = 10;
	float gridScale = kGridHalfWidth * 2.0f / (float)kSubdivision;

	for (int i = 0; i <= kSubdivision; ++i) {
		float offset = -kGridHalfWidth + (float)i * gridScale;

		Vector zStart = { offset, 0.0f, -kGridHalfWidth };
		Vector zEnd = { offset, 0.0f, kGridHalfWidth };
		Vector xStart = { -kGridHalfWidth, 0.0f, offset };
		Vector xEnd = { kGridHalfWidth, 0.0f, offset };

		Vector pZStart = Transform3DTo2D(zStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pZEnd = Transform3DTo2D(zEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pXStart = Transform3DTo2D(xStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
		Vector pXEnd = Transform3DTo2D(xEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

		uint32_t color = (offset == 0.0f) ? 0xFFFFFFFF : 0x888888FF;

		Novice::DrawLine((int)pZStart.x, (int)pZStart.y, (int)pZEnd.x, (int)pZEnd.y, color);
		Novice::DrawLine((int)pXStart.x, (int)pXStart.y, (int)pXEnd.x, (int)pXEnd.y, color);
	}
}

void DrawPlane(Plane plane, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector ext1, ext2;
	if (fabsf(plane.normal.x) > 0.9f) {
		ext1 = { 0.0f, 1.0f, 0.0f };
	}
	else {
		ext1 = { 1.0f, 0.0f, 0.0f };
	}
	ext1 = Normalize(Subtract(ext1, Multiply(Dot(ext1, plane.normal), plane.normal)));
	ext2 = {
		plane.normal.y * ext1.z - plane.normal.z * ext1.y,
		plane.normal.z * ext1.x - plane.normal.x * ext1.z,
		plane.normal.x * ext1.y - plane.normal.y * ext1.x
	};

	Vector center = Multiply(plane.distance, plane.normal);
	float size = 2.0f;

	Vector v0 = Add(center, Add(Multiply(-size, ext1), Multiply(-size, ext2)));
	Vector v1 = Add(center, Add(Multiply(size, ext1), Multiply(-size, ext2)));
	Vector v2 = Add(center, Add(Multiply(size, ext1), Multiply(size, ext2)));
	Vector v3 = Add(center, Add(Multiply(-size, ext1), Multiply(size, ext2)));

	Vector p0 = Transform3DTo2D(v0, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p1 = Transform3DTo2D(v1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p2 = Transform3DTo2D(v2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p3 = Transform3DTo2D(v3, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, color);
	Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
	Novice::DrawLine((int)p2.x, (int)p2.y, (int)p3.x, (int)p3.y, color);
	Novice::DrawLine((int)p3.x, (int)p3.y, (int)p0.x, (int)p0.y, color);

	Vector normalEnd = Add(center, Multiply(0.4f, plane.normal));
	Vector pCenter = Transform3DTo2D(center, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pNormalEnd = Transform3DTo2D(normalEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Novice::DrawLine((int)pCenter.x, (int)pCenter.y, (int)pNormalEnd.x, (int)pNormalEnd.y, 0xFF00FFFF);
}

void DrawTriangle(Triangle triangle, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector p0 = Transform3DTo2D(triangle.v0, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p1 = Transform3DTo2D(triangle.v1, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector p2 = Transform3DTo2D(triangle.v2, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)p0.x, (int)p0.y, (int)p1.x, (int)p1.y, color);
	Novice::DrawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y, color);
	Novice::DrawLine((int)p2.x, (int)p2.y, (int)p0.x, (int)p0.y, color);
}

void DrawSegment(Segment segment, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	Vector start = segment.origin;
	Vector end = Add(segment.origin, segment.diff);

	Vector pStart = Transform3DTo2D(start, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pEnd = Transform3DTo2D(end, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)pStart.x, (int)pStart.y, (int)pEnd.x, (int)pEnd.y, color);
}

void DrawIntersectionPoint(Vector point, Matrix4x4 viewMatrix, Matrix4x4 projectionMatrix, Matrix4x4 viewportMatrix, uint32_t color) {
	float size = 0.05f;
	Vector xStart = Add(point, { -size, 0, 0 }), xEnd = Add(point, { size, 0, 0 });
	Vector yStart = Add(point, { 0, -size, 0 }), yEnd = Add(point, { 0, size, 0 });
	Vector zStart = Add(point, { 0, 0, -size }), zEnd = Add(point, { 0, 0, size });

	Vector pXs = Transform3DTo2D(xStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pXe = Transform3DTo2D(xEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pYs = Transform3DTo2D(yStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pYe = Transform3DTo2D(yEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pZs = Transform3DTo2D(zStart, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);
	Vector pZe = Transform3DTo2D(zEnd, MakeIdentity(), viewMatrix, projectionMatrix, viewportMatrix);

	Novice::DrawLine((int)pXs.x, (int)pXs.y, (int)pXe.x, (int)pXe.y, color);
	Novice::DrawLine((int)pYs.x, (int)pYs.y, (int)pYe.x, (int)pYe.y, color);
	Novice::DrawLine((int)pZs.x, (int)pZs.y, (int)pZe.x, (int)pZe.y, color);
}

// --- メイン関数 ---
const char kWindowTitle[] = "3次元衝突判定（平面・三角形と線分）";

int WINAPI WinMain(_In_ HINSTANCE, _In_opt_ HINSTANCE, _In_ LPSTR, _In_ int) {

	const int kScreenWidth = 1280;
	const int kScreenHeight = 720;
	Novice::Initialize(kWindowTitle, kScreenWidth, kScreenHeight);

	// カメラ設定
	Vector cameraScale = { 1.0f, 1.0f, 1.0f };
	Vector cameraRotate = { 0.26f, 0.0f, 0.0f };
	Vector cameraTranslate = { 0.0f, 1.5f, -5.0f };

	// 線分(Segment)の設定
	Segment segment = {
		{ 0.0f, 1.0f, 0.0f },  // 始点
		{ 0.0f, -1.5f, 0.0f }  // 差分ベクトル
	};

	// 平面の設定
	Plane plane = {
		Normalize({ 0.0f, 1.0f, 0.0f }),
		0.0f
	};

	// 三角形の設定
	Triangle triangle = {
		{ 0.0f,  0.5f, 0.0f }, // 頂点0
		{ 0.5f, -0.5f, 0.0f }, // 頂点1
		{-0.5f, -0.5f, 0.0f }  // 頂点2
	};

	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	while (Novice::ProcessMessage() == 0) {
		Novice::BeginFrame();

		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		// 1. 平面と線分の衝突判定
		float segmentT = 0.0f;
		Vector intersectionPoint = { 0, 0, 0 };
		bool isSegmentColliding = IsCollisionPlaneToSegment(plane, segment, segmentT, intersectionPoint);

		// 2. 三角形と線分の衝突判定
		float triangleT = 0.0f;
		Vector triangleIntersectionPoint = { 0, 0, 0 };
		bool isTriangleColliding = IsCollisionTriangleToSegment(triangle, segment, triangleT, triangleIntersectionPoint);

		// 衝突状態による色の切り替え
		uint32_t colorSegment = isTriangleColliding ? 0xFF0000FF : (isSegmentColliding ? 0xFFFF00FF : 0x00FF00FF);
		uint32_t colorPlane = isSegmentColliding ? 0x880000FF : 0xFFFFFFFF;
		uint32_t colorTriangle = isTriangleColliding ? 0xFF0000FF : 0xFFFFFFFF;

		// --- ImGuiによるコントロールパネル ---
		ImGui::Begin("Collision Control Panel");

		if (ImGui::CollapsingHeader("Camera Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Camera Pos", &cameraTranslate.x, 0.05f);
			ImGui::DragFloat3("Camera Rot", &cameraRotate.x, 0.01f);
		}

		if (ImGui::CollapsingHeader("Plane Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			if (ImGui::DragFloat3("Plane Normal", &plane.normal.x, 0.01f, -1.0f, 1.0f)) {
				plane.normal = Normalize(plane.normal);
			}
			ImGui::DragFloat("Plane Distance", &plane.distance, 0.02f, -5.0f, 5.0f);
		}

		if (ImGui::CollapsingHeader("Triangle Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Vertex 0", &triangle.v0.x, 0.02f);
			ImGui::DragFloat3("Vertex 1", &triangle.v1.x, 0.02f);
			ImGui::DragFloat3("Vertex 2", &triangle.v2.x, 0.02f);
		}

		if (ImGui::CollapsingHeader("Segment Control", ImGuiTreeNodeFlags_DefaultOpen)) {
			ImGui::DragFloat3("Segment Origin(Start)", &segment.origin.x, 0.02f);
			ImGui::DragFloat3("Segment Diff", &segment.diff.x, 0.02f);
		}

		ImGui::Separator();

		// 結果表示 (平面)
		ImGui::Text("--- Plane to Segment ---");
		ImGui::Text("Intersect t: %.4f", segmentT);
		if (isSegmentColliding) {
			ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "STATUS: PLANE COLLIDING!");
			ImGui::Text("Point: (%.2f, %.2f, %.2f)", intersectionPoint.x, intersectionPoint.y, intersectionPoint.z);
		}
		else {
			ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "STATUS: NO PLANE COLLISION");
		}

		// 結果表示 (三角形)
		ImGui::Text("--- Triangle to Segment ---");
		ImGui::Text("Intersect t: %.4f", triangleT);
		if (isTriangleColliding) {
			ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "STATUS: TRIANGLE COLLIDING!");
			ImGui::Text("Point: (%.2f, %.2f, %.2f)", triangleIntersectionPoint.x, triangleIntersectionPoint.y, triangleIntersectionPoint.z);
		}
		else {
			ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f), "STATUS: NO TRIANGLE COLLISION");
		}

		ImGui::End();

		// 行列の生成
		Matrix4x4 cameraWorldMatrix = MakeAffineMatrix(cameraScale, cameraRotate, cameraTranslate);
		Matrix4x4 viewMatrix = Inverse(cameraWorldMatrix);
		Matrix4x4 projectionMatrix = MakePerspectiveFovMatrix(0.45f, (float)kScreenWidth / (float)kScreenHeight, 0.1f, 100.0f);
		Matrix4x4 viewportMatrix = MakeViewportMatrix(0.0f, 0.0f, (float)kScreenWidth, (float)kScreenHeight, 0.0f, 1.0f);

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		// 1. グリッド
		DrawGrid(viewMatrix, projectionMatrix, viewportMatrix);

		// 2. 平面
		DrawPlane(plane, viewMatrix, projectionMatrix, viewportMatrix, colorPlane);

		// 3. 三角形
		DrawTriangle(triangle, viewMatrix, projectionMatrix, viewportMatrix, colorTriangle);

		// 4. 線分
		DrawSegment(segment, viewMatrix, projectionMatrix, viewportMatrix, colorSegment);

		// 5. 交点の描画
		if (isTriangleColliding) {
			// 三角形との交点はシアン色(水色)
			DrawIntersectionPoint(triangleIntersectionPoint, viewMatrix, projectionMatrix, viewportMatrix, 0x00FFFFFF);
		}
		else if (isSegmentColliding) {
			// 平面のみとの交点はイエロー(黄色)
			DrawIntersectionPoint(intersectionPoint, viewMatrix, projectionMatrix, viewportMatrix, 0xFFFF00FF);
		}

		///
		/// ↑描画処理ここまで
		///

		Novice::EndFrame();

		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	Novice::Finalize();
	return 0;
}