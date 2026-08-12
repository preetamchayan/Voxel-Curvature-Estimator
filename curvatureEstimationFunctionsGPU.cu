/*Curvature estimation for voxelized object ---> GPU implementation*/

#include "voxelizeGPU.cu"
#include<time.h>

//global variables
int local_max = INT_MIN, local_min = INT_MAX, global_max = INT_MIN, global_min = INT_MAX, avg_max = INT_MIN, avg_min = INT_MAX;

//creating voxel faces for the point coordinates
void point2Voxel(FILE *fp, int x, int y, int z, int k){
	fprintf(fp, "v %d %d %d\n", k*x, k*y, k*(z+1));
	fprintf(fp, "v %d %d %d\n", k*(x+1), k*y, k*(z+1));
	fprintf(fp, "v %d %d %d\n", k*(x+1), k*(y+1), k*(z+1));
	fprintf(fp, "v %d %d %d\n", k*x, k*(y+1), k*(z+1));
	fprintf(fp, "v %d %d %d\n", k*(x+1), k*(y+1), k*z);
	fprintf(fp, "v %d %d %d\n", k*(x+1), k*y, k*z);
	fprintf(fp, "v %d %d %d\n", k*x, k*y, k*z);
	fprintf(fp, "v %d %d %d\n", k*x, k*(y+1), k*z);
	fprintf(fp, "f -8 -7 -6 -5\n");
	fprintf(fp, "f -4 -3 -2 -1\n");
	fprintf(fp, "f -3 -4 -6 -7\n");
	fprintf(fp, "f -2 -8 -5 -1\n");
	fprintf(fp, "f -2 -3 -7 -8\n");
	fprintf(fp, "f -1 -5 -6 -4\n");
}

//hsv to rgb color space conversion
float* hsv2rgb(float *hsv){
	float *rgb = (float*)malloc(3*sizeof(float));
	hsv[0] /= 360;
	int i;
	float aa, bb, cc, f;
	if(hsv[1] == 0) rgb[0] = rgb[1] = rgb[2] = hsv[2];
	else{
		hsv[0] *= 6.0;
		i = floor(hsv[0]);
		f = hsv[0]-i;
		aa = hsv[2]*(1-hsv[1]);
		bb = hsv[2]*(1-(hsv[1]*f));
		cc = hsv[2]*(1-(hsv[1]*(1-f)));
		switch(i){
			case 0: rgb[0] = hsv[2]; rgb[1] = cc; rgb[2] = aa; break;
			case 1: rgb[0] = bb; rgb[1] = hsv[2]; rgb[2] = aa; break;
			case 2: rgb[0] = aa; rgb[1] = hsv[2]; rgb[2] = cc; break;
			case 3: rgb[0] = aa; rgb[1] = bb; rgb[2] = hsv[2]; break;
			case 4: rgb[0] = cc; rgb[1] = aa; rgb[2] = hsv[2]; break;
			case 5: rgb[0] = hsv[2]; rgb[1] = aa; rgb[2] = bb; break;
		}
	}
	return rgb;
}

//compute color of voxels based on their curvature values
unsigned char **voxelColor(FILE *fp){
	unsigned char **rgb_color = (unsigned char**)malloc((global_max+2)*sizeof(unsigned char*));
	for(int i = 0; i <= global_max+1; i++) rgb_color[i] = (unsigned char*)malloc(3*sizeof(unsigned char));
	rgb_color[0][0] = 128; rgb_color[0][1] = 128; rgb_color[0][2] = 128;
	for(int i = 1; i <= global_max+1; i++){
		float *hsv = (float*)malloc(3*sizeof(float));
		hsv[0] = 240.0-((240.0)*((float)(i-1)))/(float)(global_max);
		hsv[1] = /*(float)((i-1))/(float)(global_max)*/1; hsv[2] = 1;
		float *rgb = hsv2rgb(hsv);
		fprintf(fp, "newmtl %d\nKd %.4f %.4f %.4f\nillum 1\n\n", i, rgb[0], rgb[1], rgb[2]);
		rgb_color[i][0] = (int)(rgb[0]*255); rgb_color[i][1] = (int)(rgb[1]*255); rgb_color[i][2] = (int)(rgb[2]*255);
		free(hsv);
		free(rgb);
	}
	return rgb_color;
}

//GPU code to identify inner voxels without marking
__global__ void innerSpaceGPU(unsigned char *obj, int R, int C, int D, int plane){

	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;
	
	if(x >= R || y >= C) return;
	
	int id, W;
	switch(plane){
		case 0: id = x*D+y*R*D; W=1; break;
		case 1: id = x*C*D+y; W=C; break;
		case 2: id = x+y*R; W=R*C; break;
	}
	
	int count = 0, z;
	for(z = 0; z < D; z++){
		if(obj[id+z*W] == 1){
			if(z == D-1 || obj[id+(z+1)*W] == 1) continue;
			else if(obj[id+(z+1)*W] != 1) count++;
		}
		else if(count%2 == 1) obj[id+z*W] += 2;
	}
	if(count%2 == 1){
		z--;
		while(obj[id+z*W] != 1){
			obj[id+z*W] -= 2;
			z--;
		}
	}
}

//GPU code to mark inner voxels
__global__ void markInterior(unsigned char *obj, int R, int C, int D){

	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	
	if(x >= R || y >= C || z >= D) return;
	
	int id = x+y*R+z*R*C;
	
	if(obj[id] == 2 || obj[id] == 4) obj[id] = 0;
	if(obj[id] == 6) obj[id] = 2;
	
}

//GPU code to identify frontier voxels
__global__ void identifyFrontier(unsigned char *obj, int R, int C, int D){

	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	
	if(x >= R || y >= C || z >= D) return;
	
	int id = x+y*R+z*R*C;
	
	if(obj[id] != 1) return;

	if(x == 0 || obj[id-1] == 0) return;
	if(x == R-1 || obj[id+1] == 0) return;
	if(y == 0 || obj[id-R] == 0) return;
	if(y == C-1 || obj[id+R] == 0) return;
	if(z == 0 || obj[id-R*C] == 0) return;
	if(z == D-1 || obj[id+R*C] == 0) return;
	obj[id] = 2;
}

//mark the voxles present inside the object (not visible from outside)
void frontierVoxel(){

	printf("\nComputing inner space...\n");
	
	int dim = 8;
	dim3 block = dim3(dim, dim);
	dim3 grid;
	
	//a ray along z-axis
	grid = dim3((R+dim-1)/dim, (C+dim-1)/dim);
	innerSpaceGPU<<<grid, block>>>(obj, R, C, D, 2);
	cudaDeviceSynchronize();
	
	//a ray along y-axis
	grid = dim3((D+dim-1)/dim, (R+dim-1)/dim);
	innerSpaceGPU<<<grid, block>>>(obj, D, R, C, 1);
	cudaDeviceSynchronize();
	
	//a ray along x-axis
	grid = dim3((C+dim-1)/dim, (D+dim-1)/dim);
	innerSpaceGPU<<<grid, block>>>(obj, C, D, R, 0);
	cudaDeviceSynchronize();
	
	//mark the interior
	block = dim3(dim, dim, dim);
	grid = dim3((R+dim-1)/dim, (C+dim-1)/dim, (D+dim-1)/dim);
	markInterior<<<grid, block>>>(obj, R, C, D);
	cudaDeviceSynchronize();
	
	//frontier voxel identification
	identifyFrontier<<<grid, block>>>(obj, R, C, D);
	cudaDeviceSynchronize();
	for(int id = 0; id < V; id++) if(obj[id] == 1) vc++;

}

//averaging operation for local curvature adjustment
void averageCurvature(FILE *fp_obj, FILE *fp, FILE *fp_mtl, unsigned char *voxel){
	int vx = 0;
	printf("\nAveraging curvature...\n");
	for(int id = 0; id < V; id++){
		if(obj[id] == 1){
			int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R;
			int curve_value = 0, count = 0;
			if(curvature[id] != 255 && curvature[id+V] != 255){
				curve_value += (curvature[id] + curvature[id+V]);
				local_max = max(local_max, curve_value);
				local_min = min(local_min, curve_value);
				count++;
			}
			for(int i = -2; i <= 2; i++){
				for(int j = -2; j <= 2; j++){
					for(int k = -2; k <= 2; k++){
						if(i == 0 && j == 0 && k == 0 || x+i <= -1 || x+i >= R || y+j <= -1 || y+j >= C || z+k <= -1 || z+k >= D) continue;
						if(obj[x+i+(y+j)*R+(z+k)*R*C] == 1 && curvature[x+i+(y+j)*R+(z+k)*R*C] != 255 && curvature[x+i+(y+j)*R+(z+k)*R*C+V] != 255){
							curve_value += (curvature[x+i+(y+j)*R+(z+k)*R*C] + curvature[x+i+(y+j)*R+(z+k)*R*C+V]); 
							count++;
						}
					}
				}
			}
			if(count == 0) voxel[vx++] = 255;
			else{
				curve_value /= count;
				avg_max = max(avg_max, curve_value);
				avg_min = min(avg_min, curve_value);
				voxel[vx++] = min(global_max, curve_value);
			}
		}
	}
	
	global_max = min(global_max, avg_max);
	unsigned char **rgb = voxelColor(fp_mtl);
	fclose(fp_mtl);
	
	vx = 0;
	for(int id = 0; id < V; id++) if(obj[id] == 1){
		int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R;
		int color = voxel[vx]==255?-1:voxel[vx]; vx++;
		fprintf(fp_obj, "\nusemtl %d\n\n", color+1);
		fprintf(fp, "%d %d %d %d %d %d %d\n", x, y, z, rgb[color+1][0], rgb[color+1][1], rgb[color+1][2], color);
		point2Voxel(fp_obj, x, y, z, 1);
	}
	for(int i = 0; i < global_max+2; i++) free(rgb[i]); free(rgb);
}

//chain code computation
__device__ int getChainCode(int x, int y, int prev_x, int prev_y){
	if(x-prev_x == 1 && y-prev_y == 0) return 0;
	if(x-prev_x == 1 && y-prev_y == 1) return 1;
	if(x-prev_x == 0 && y-prev_y == 1) return 2;
	if(x-prev_x == -1 && y-prev_y == 1) return 3;
	if(x-prev_x == -1 && y-prev_y == 0) return 4;
	if(x-prev_x == -1 && y-prev_y == -1) return 5;
	if(x-prev_x == 0 && y-prev_y == -1) return 6;
	if(x-prev_x == 1 && y-prev_y == -1) return 7;
	return -1;
}

//add id1 as the i-th child of id
__device__ void addChild(int id, int id1, int i, int plane_no, int child[8][2], int R, int C, int D){
	int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R;
	t = id1%(R*C); int z1 = id1/(R*C), y1 = t/R, x1 = t-y1*R;
	child[i][0] = id1;
	switch(plane_no/3){
		case 0: child[i][1] = getChainCode(x1, y1, x, y); break;
		case 1: child[i][1] = getChainCode(y1, z1, y, z); break;
		case 2: child[i][1] = getChainCode(z1, x1, z, x); break;
	}
}

//finds the neighbours of a voxel which lie on a particular plane as well as on the object
__device__ void getNeighbor(unsigned char* obj, int id, int plane_no, int child[8][2], int R, int C, int D){
	int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R, W = R*C;
	switch(plane_no){
		case 0: // xy plane zero degree
			if(x != R-1 && obj[id+1] == 1) addChild(id, id+1, 0, plane_no, child, R, C, D);
			if(x != 0 && obj[id-1] == 1) addChild(id, id-1, 1, plane_no, child, R, C, D); 
			if(y != C-1 && obj[id+R] == 1) addChild(id, id+R, 2, plane_no, child, R, C, D); 
			if(y != 0 && obj[id-R] == 1) addChild(id, id-R, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && obj[id+1+R] == 1) addChild(id, id+1+R, 4, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && obj[id+1-R] == 1) addChild(id, id+1-R, 5, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && obj[id-1+R] == 1) addChild(id, id-1+R, 6, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && obj[id-1-R] == 1) addChild(id, id-1-R, 7, plane_no, child, R, C, D);
		break;
		case 1: // xy plane 45 degree anti-clockwise
			if(y != C-1 && (obj[id+R] == 1 ||
			(x != 0 && obj[id-1+R] == 1 || x != R-1 && obj[id+1+R] == 1) &&
			(z != 0 && obj[id+R-W] == 1 || z != D-1 && obj[id+R+W] == 1) && obj[id+R] != 2)) 
				addChild(id, id+R, 0, plane_no, child, R, C, D);
			if(y != 0 && (obj[id-R] == 1 ||
			(x != 0 && obj[id-1-R] == 1 || x != R-1 && obj[id+1-R] == 1) &&
			(z != 0 && obj[id-R-W] == 1 || z != D-1 && obj[id-R+W] == 1) && obj[id-R] != 2))
				addChild(id, id-R, 1, plane_no, child, R, C, D);
			if(x != R-1 && z != 0 && (obj[id+1-W] == 1))
				addChild(id, id+1-W, 2, plane_no, child, R, C, D);
			if(x != 0 && z != D-1 && (obj[id-1+W] == 1))
				addChild(id, id-1+W, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != 0 && (obj[id+1+R-W] == 1 ||
			obj[id+R-W] == 1 && obj[id+1+R] == 1 && obj[id+1+R-W] != 2))
				addChild(id, id+1+R-W, 4, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != D-1 && (obj[id-1+R+W] == 1 ||
			obj[id+R+W] == 1 && obj[id-1+R] == 1 && obj[id-1+R+W] != 2))
				addChild(id, id-1+R+W, 5, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != 0 && (obj[id+1-R-W] == 1 ||
			obj[id-R-W] == 1 && obj[id+1-R] == 1 && obj[id+1-R-W] != 2))
				addChild(id, id+1-R-W, 6, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != D-1 && (obj[id-1-R+W] == 1 ||
			obj[id-R+W] == 1 && obj[id-1-R] == 1 && obj[id-1-R+W] != 2))
				addChild(id, id-1-R+W, 7, plane_no, child, R, C, D);
		break;
		case 2: // xy plane 45 degree clockwise
			if(y != C-1 && (obj[id+R] == 1 ||
			(x != 0 && obj[id-1+R] == 1 || x != R-1 && obj[id+1+R] == 1) &&
			(z != 0 && obj[id+R-W] == 1 || z != D-1 && obj[id+R+W] == 1) && obj[id+R] != 2))
				addChild(id, id+R, 0, plane_no, child, R, C, D);
			if(y != 0 && (obj[id-R] == 1 ||
			(x != 0 && obj[id-1-R] == 1 || x != R-1 && obj[id+1-R] == 1) &&
			(z != 0 && obj[id-R-W] == 1 || z != D-1 && obj[id-R+W] == 1) && obj[id-R] != 2))
				addChild(id, id-R, 1, plane_no, child, R, C, D);
			if(x != R-1 && z != D-1 && (obj[id+1+W] == 1))
				addChild(id, id+1+W, 2, plane_no, child, R, C, D);
			if(x != 0 && z != 0 && (obj[id-1-W] == 1)) 
				addChild(id, id-1-W, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != D-1 && (obj[id+1+R+W] == 1 ||
			obj[id+R+W] == 1 && obj[id+1+R] == 1 && obj[id+1+R+W] != 2))
				addChild(id, id+1+R+W, 4, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != D-1 && (obj[id+1-R+W] == 1 ||
			obj[id-R+W] == 1 && obj[id+1-R] == 1 && obj[id+1-R+W] != 2))
				addChild(id, id+1-R+W, 5, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != 0 && (obj[id-1+R-W] == 1 || 
			obj[id-1+R] == 1 && obj[id+R-W] == 1 && obj[id-1+R-W] != 2))
				addChild(id, id-1+R-W, 6, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != 0 && (obj[id-1-R-W] == 1 || 
			obj[id-R-W] == 1 && obj[id-1-R] == 1 && obj[id-1-R-W] != 2))
				addChild(id, id-1-R-W, 7, plane_no, child, R, C, D);
		break;
		case 3: // yz plane zero degree
			if(y != C-1 && obj[id+R] == 1) addChild(id, id+R, 0, plane_no, child, R, C, D);
			if(y != 0 && obj[id-R] == 1) addChild(id, id-R, 1, plane_no, child, R, C, D);
			if(z != D-1 && obj[id+W] == 1) addChild(id, id+W, 2, plane_no, child, R, C, D);
			if(z != 0 && obj[id-W] == 1) addChild(id, id-W, 3, plane_no, child, R, C, D);
			if(y != C-1 && z != D-1 && obj[id+R+W] == 1) addChild(id, id+R+W, 4, plane_no, child, R, C, D);
			if(y != C-1 && z != 0 && obj[id+R-W] == 1) addChild(id, id+R-W, 5, plane_no, child, R, C, D);
			if(y != 0 && z != D-1 && obj[id-R+W] == 1) addChild(id, id-R+W, 6, plane_no, child, R, C, D);
			if(y != 0 && z != 0 && obj[id-R-W] == 1) addChild(id, id-R-W, 7, plane_no, child, R, C, D);
		break;
		case 4: // yz plane 45 degree anti-clockwise
			if(z != D-1 && (obj[id+W] == 1 ||
			(x != 0 && obj[id-1+W] == 1 || x != R-1 && obj[id+1+W] == 1) && 
			(y != 0 && obj[id-R+W] == 1 || y != C-1 && obj[id+R+W] == 1) && obj[id+W] != 2))
				addChild(id, id+W, 0, plane_no, child, R, C, D);
			if(z != 0 && (obj[id-W] == 1 ||
			(x != 0 && obj[id-1-W] == 1 || x != R-1 && obj[id+1-W] == 1) && 
			(y != 0 && obj[id-R-W] == 1 || y != C-1 && obj[id+R-W] == 1) && obj[id-W] != 2))
				addChild(id, id-W, 1, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && (obj[id-1+R] == 1)) 
				addChild(id, id-1+R, 2, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && (obj[id+1-R] == 1))
				addChild(id, id+1-R, 3, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != D-1 && (obj[id-1+R+W] == 1 ||
			obj[id+R+W] == 1 && obj[id-1+W] == 1 && obj[id-1+R+W] != 2)) 
				addChild(id, id-1+R+W, 4, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != 0 && (obj[id-1+R-W] == 1 ||
			obj[id+R-W] == 1 && obj[id-1-W] == 1 && obj[id-1+R-W] != 2))
				addChild(id, id-1+R-W, 5, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != D-1 && (obj[id+1-R+W] == 1 ||
			obj[id-R+W] == 1 && obj[id+1+W] == 1 && obj[id+1-R+W] != 2))
				addChild(id, id+1-R+W, 6, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != 0 && (obj[id+1-R-W] == 1 ||
			obj[id-R-W] == 1 && obj[id+1-W] == 1 && obj[id+1-R-W] != 2))
				addChild(id, id+1-R-W, 7, plane_no, child, R, C, D);
		break;
		case 5: // yz plane 45 degree clockwise
			if(z != D-1 && (obj[id+W] == 1 ||
			(x != 0 && obj[id-1+W] == 1 || x != R-1 && obj[id+1+W] == 1) &&
			(y != 0 && obj[id-R+W] == 1 || y != C-1 && obj[id+R+W] == 1) && obj[id+W] != 2)) 
				addChild(id, id+W, 0, plane_no, child, R, C, D);
			if(z != 0 && (obj[id-W] == 1 ||
			(x != 0 && obj[id-1-W] == 1 || x != R-1 && obj[id+1-W] == 1) && 
			(y != 0 && obj[id-R-W] == 1 || y != C-1 && obj[id+R-W] == 1) && obj[id-W] != 2)) 
				addChild(id, id-W, 1, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && (obj[id+1+R] == 1))
				addChild(id, id+1+R, 2, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && (obj[id-1-R] == 1)) 
				addChild(id, id-1-R, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != D-1 && (obj[id+1+R+W] == 1 ||
			obj[id+R+W] == 1 && obj[id+1+W] == 1 && obj[id+1+R+W] != 2)) 
				addChild(id, id+1+R+W, 4, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != 0 && (obj[id+1+R-W] == 1 ||
			obj[id+1-W] == 1 && obj[id+R-W] == 1 && obj[id+1+R-W] != 2)) 
				addChild(id, id+1+R-W, 5, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != D-1 && (obj[id-1-R+W] == 1 ||
			obj[id-R+W] == 1 && obj[id-1+W] == 1 && obj[id-1-R+W] != 2)) 
				addChild(id, id-1-R+W, 6, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != 0 && (obj[id-1-R-W] == 1 ||
			obj[id-R-W] == 1 && obj[id-1-W] == 1 && obj[id-1-R-W] != 2)) 
				addChild(id, id-1-R-W, 7, plane_no, child, R, C, D);
		break;
		case 6: // zx plane zero degree
			if(z != D-1 && obj[id+W] == 1) addChild(id, id+W, 0, plane_no, child, R, C, D);
			if(z != 0 && obj[id-W] == 1) addChild(id, id-W, 1, plane_no, child, R, C, D);
			if(x != R-1 && obj[id+1] == 1) addChild(id, id+1, 2, plane_no, child, R, C, D);
			if(x != 0 && obj[id-1] == 1) addChild(id, id-1, 3, plane_no, child, R, C, D);
			if(x != R-1 && z != D-1 && obj[id+1+W] == 1) addChild(id, id+1+W, 4, plane_no, child, R, C, D);
			if(x != 0 && z != D-1 && obj[id-1+W] == 1) addChild(id, id-1+W, 5, plane_no, child, R, C, D);
			if(x != R-1 && z != 0 && obj[id+1-W] == 1) addChild(id, id+1-W, 6, plane_no, child, R, C, D);
			if(x != 0 && z != 0 && obj[id-1-W] == 1) addChild(id, id-1-W, 7, plane_no, child, R, C, D);
		break;
		case 7: // zx plane 45 degree anti-clockwise
			if(x != R-1 && (obj[id+1] == 1 ||
			(y != 0 && obj[id+1-R] == 1 || y != C-1 && obj[id+1+R] == 1) && 
			(z != 0 && obj[id+1-W] == 1 || z != D-1 && obj[id+1+W] == 1) && obj[id+1] != 2)) 
				addChild(id, id+1, 0, plane_no, child, R, C, D);
			if(x != 0 && (obj[id-1] == 1 ||
			(y != 0 && obj[id-1-R] == 1 || y != C-1 && obj[id-1+R] == 1) && 
			(z != 0 && obj[id-1-W] == 1 || z != D-1 && obj[id-1+W] == 1) && obj[id-1] != 2)) 
				addChild(id, id-1, 1, plane_no, child, R, C, D);
			if(y != 0 && z != D-1 && (obj[id-R+W] == 1)) 
				addChild(id, id-R+W, 2, plane_no, child, R, C, D);
			if(y != C-1 && z != 0 && (obj[id+R-W] == 1)) 
				addChild(id, id+R-W, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != D-1 && (obj[id+1-R+W] == 1 ||
			obj[id+1+W] == 1 && obj[id+1-R] == 1 && obj[id+1-R+W] != 2)) 
				addChild(id, id+1-R+W, 4, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != D-1 && (obj[id-1-R+W] == 1 ||
			obj[id-1+W] == 1 && obj[id-1-R] == 1 && obj[id-1-R+W] != 2)) 
				addChild(id, id-1-R+W, 5, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != 0 && (obj[id+1+R-W] == 1 ||
			obj[id+1-W] == 1 && obj[id+1+R] == 1 && obj[id+1+R-W] != 2)) 
				addChild(id, id+1+R-W, 6, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != 0 && (obj[id-1+R-W] == 1 ||
			obj[id-1-W] == 1 && obj[id-1+R] == 1 && obj[id-1+R-W] != 2)) 
				addChild(id, id-1+R-W, 7, plane_no, child, R, C, D);
		break;
		case 8: // zx plnae 45 degree clockwise
			if(x != R-1 && (obj[id+1] == 1 ||
			(y != 0 && obj[id+1-R] == 1 || y != C-1 && obj[id+1+R] == 1) && 
			(z != 0 && obj[id+1-W] == 1 || z != D-1 && obj[id+1+W] == 1) && obj[id+1] != 2)) 
				addChild(id, id+1, 0, plane_no, child, R, C, D);
			if(x != 0 && (obj[id-1] == 1 ||
			(y != 0 && obj[id-1-R] == 1 || y != C-1 && obj[id-1+R] == 1) && 
			(z != 0 && obj[id-1-W] == 1 || z != D-1 && obj[id-1+W] == 1) && obj[id-1] != 2)) 
				addChild(id, id-1, 1, plane_no, child, R, C, D);
			if(y != C-1 && z != D-1 && (obj[id+R+W] == 1)) 
				addChild(id, id+R+W, 2, plane_no, child, R, C, D);
			if(y != 0 && z != 0 && (obj[id-R-W] == 1)) 
				addChild(id, id-R-W, 3, plane_no, child, R, C, D);
			if(x != R-1 && y != C-1 && z != D-1 && (obj[id+1+R+W] == 1 ||
			obj[id+1+W] == 1 && obj[id+1+R] == 1 && obj[id+1+R+W] != 2)) 
				addChild(id, id+1+R+W, 4, plane_no, child, R, C, D);
			if(x != R-1 && y != 0 && z != 0 && (obj[id+1-R-W] == 1 ||
			obj[id+1-W] == 1 && obj[id+1-R] == 1 && obj[id+1-R-W] != 2)) 
				addChild(id, id+1-R-W, 5, plane_no, child, R, C, D);
			if(x != 0 && y != C-1 && z != D-1 && (obj[id-1+R+W] == 1 ||
			obj[id-1+W] == 1 && obj[id-1+R] == 1 && obj[id-1+R+W] != 2)) 
				addChild(id, id-1+R+W, 6, plane_no, child, R, C, D);
			if(x != 0 && y != 0 && z != 0 && (obj[id-1-R-W] == 1 ||
			obj[id-1-W] == 1 && obj[id-1-R] == 1 && obj[id-1-R-W] != 2)) 
				addChild(id, id-1-R-W, 7, plane_no, child, R, C, D);
	}
}

//get the (i+1)-th voxel in the leading/trailing curve
__device__ int getNextNeighbor(unsigned char *obj, unsigned int curve[], int plane_no, int arr[8][2], int R, int C, int D, int i, int k, int flag){
	int id = curve[k+flag*i];
	for(int j = 0; j < 8; j++) arr[j][0] = -1;
	getNeighbor(obj, id, plane_no, arr, R, C, D);
	int cnt = 0;
	for(int j = 0; j < 8; j++){ 
		for(int p = k-i; p <= k+i; p++){
			if(curve[p] == arr[j][0]){
				arr[j][0] = -1; arr[j][1] = -1;
				break;
			}
		}
		if(arr[j][0] != -1) cnt++;
		if(cnt > 3) return cnt;
	}
	
	return cnt;
}

//produce chain code sequence and estimate 2D curvature
__device__ int estimate2DCurvature(unsigned char *obj, int id, int plane_no, int k, int R, int C, int D){
	unsigned int curve[129];
	curve[k] = id;
	int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R;
	unsigned char trail_cc, lead_cc, prev_tcc, prev_lcc;
	int curvature = INT_MAX;
	int sum = 0, i, cnt = 0;
	int head[8][2], trail[8][2], lead[8][2];
	
	for(int j = 0; j < 8; j++) head[j][0] = -1;
	getNeighbor(obj, id, plane_no, head, R, C, D); //find neighbors of the point of interest
	for(int j = 0; j < 8; j++) if(head[j][0] != -1) cnt++;
	if(cnt == 0 || cnt > 3) return -1;
	
	for(int m = 0; m < 7; m++){
		if(head[m][0] == -1) continue;
		for(int n = m+1; n < 8; n++){
			if(head[n][0] == -1) continue;
			trail[0][0] = head[m][0], trail[0][1] = head[m][1];
			lead[0][0] = head[n][0], lead[0][1] = head[n][1];
			for(int j = 1; j < 8; j++) {trail[j][0] = -1; lead[j][0] = -1;}
			prev_tcc = 255; prev_lcc = 255;
			
			for(i = 1; i <= k; i++){
				trail_cc = 255; lead_cc = 255;
				unsigned char min_diff = 255;
				
				for(int j = 0; j < 8; j++){
					if(trail[j][0] == -1) continue;
					int id1 = trail[j][0];
					t = id1%(R*C); int z1 = id1/(R*C), y1 = t/R, x1 = t-y1*R;
					unsigned char t_cc = trail[j][1];
					for(int p = 0; p < 8; p++){
						if(lead[p][0] == -1) continue;
						int id2 = lead[p][0];
						t = id2%(R*C); int z2 = id2/(R*C), y2 = t/R, x2 = t-y2*R;
						unsigned char l_cc = lead[p][1]<4?lead[p][1]+4:lead[p][1]-4;
						int dist_x12 = max(max(abs(x1-x2), abs(y1-y2)), abs(z1-z2));
						int dist_x10 = max(max(abs(x1-x), abs(y1-y)), abs(z1-z));
						int dist_x20 = max(max(abs(x-x2), abs(y-y2)), abs(z-z2));
						if((dist_x10 == 1 || dist_x20 == 1) && i != 1) continue;
						if((dist_x10 != 1 || dist_x20 != 1) && dist_x12 <= 1) continue;
						char diff = abs(l_cc-t_cc); diff = min(diff, 8-diff);
						if(diff < min_diff){
							trail_cc = t_cc; lead_cc = l_cc; min_diff = diff;
							curve[k-i] = id1;	curve[k+i] = id2;
						}
					} //end of p loop
				} // end of j loop
				
				if(trail_cc == 255 || lead_cc == 255) break;
				if(prev_tcc != 255 && prev_lcc != 255){
					int tmp = abs(trail_cc-lead_cc); tmp = min(tmp, 8-tmp);
					int tmp1 = abs(trail_cc-prev_lcc); tmp1 = min(tmp1, 8-tmp1); tmp = min(tmp, tmp1);
					tmp1 = abs(prev_tcc-lead_cc); tmp1 = min(tmp1, 8-tmp1); tmp = min(tmp, tmp1);
					tmp1 = abs(prev_tcc-prev_lcc); tmp1 = min(tmp1, 8-tmp1);	tmp = min(tmp, tmp1);
					sum += tmp;
				}
				prev_lcc = lead_cc; prev_tcc = trail_cc;
				
				if(i == k) continue;
				
				//next trailing voxels at height k+1
				cnt = getNextNeighbor(obj, curve, plane_no, trail, R, C, D, i, k, -1);
				if(cnt == 0 || cnt > 3) break;
				
				//next leading voxels at height k+1
				cnt = getNextNeighbor(obj, curve, plane_no, lead, R, C, D, i, k, 1);
				if(cnt == 0 || cnt > 3) break;
			} // end of i loop
			
			if(i != k+1) continue;
			curvature = min(sum, curvature);
			if(curvature == 0) return 1;
		}//end of n loop
	} //end of m loop
	return (curvature == INT_MAX? -1 : curvature+1);
}

//GPU code for curvature estimation
__global__ void estimate3DCurvatureGPU(unsigned char *obj, unsigned char *curvature, int R, int C, int D, int k){
	
	int x = blockIdx.x*blockDim.x + threadIdx.x;
	int y = blockIdx.y*blockDim.y + threadIdx.y;
	int z = blockIdx.z*blockDim.z + threadIdx.z;
	
	if(x >= R || y >= C || z >= D) return;
	
	int id = x+y*R+z*R*C;
	int V = R*C*D;
	
	if(obj[id] != 1) return;
	
	int max_value = INT_MIN, min_value = INT_MAX, temp;
	for(int plane_no = 0; plane_no < 9; plane_no++){
		temp = estimate2DCurvature(obj, id, plane_no, k, R, C, D);
		if(temp == -1) continue;
		max_value = max(max_value, temp);
		min_value = min(min_value, temp);
	}
	
	if(max_value == INT_MIN || min_value == INT_MAX){
		curvature[id] = 255;
		curvature[id+V] = 255;
	}
	else{
		curvature[id] = min_value-1;
		curvature[id+V] = max_value-1;	
	}
}

//computes the 3D curvature from the 2D curvatures of nine different planes
void estimateCurvature(int k, const char *argv){
	
	//initialization
	int dim = 8;
	dim3 block = dim3(dim, dim, dim);
	dim3 grid = dim3((R+dim-1)/dim, (C+dim-1)/dim, (D+dim-1)/dim);
	
	//curvature estimation
	printf("\nCurvature estimation started...\n");
	clock_t t1 = clock();
	estimate3DCurvatureGPU<<<grid, block>>>(obj, curvature, R, C, D, k);
	cudaDeviceSynchronize();
	clock_t t2 = clock();
	printf("\nGPU time for curvature estimation: %.2f sec\n", (float)(t2-t1)/CLOCKS_PER_SEC);
	
	//averaging
	voxel = (unsigned char*)malloc(vc*sizeof(unsigned char));
	FILE *fp_obj = fileopen(argv, ".obj"); if(fp_obj == NULL) exit(1); //curvature estimation of object
	FILE *fp_mtl = fileopen(argv, ".mtl"); if(fp_mtl == NULL) exit(1); //color of curvature 
	fprintf(fp_obj, "mtllib %s.mtl", argv);
	fprintf(fp_mtl, "newmtl 0\nKd 1 1 0.5\nillum 1\n\n"); //gray color (for voxels whose curvature cannot be estimated)
	global_max = 4*(k-1); global_min = 0; //highest and lowest curvature possible
	FILE *fp = fileopen(argv, "-center-avg.txt");
	t1 = clock();
	averageCurvature(fp_obj, fp, fp_mtl, voxel);
	/*for(int id = 0; id < V; id++) if(obj[id] == 1){
		int t = id%(R*C), z = id/(R*C), y = t/R, x = t-y*R;
		fprintf(fp_obj, "\nusemtl 0\n\n");
		//fprintf(fp, "%d %d %d %d %d %d %d\n", x, y, z, rgb[color+1][0], rgb[color+1][1], rgb[color+1][2], color);
		point2Voxel(fp_obj, x, y, z, 1);
	}*/
	t2 = clock();
	fclose(fp);
	//printf("\nCPU time for averaging: %.2f sec\n", (float)(t2-t1)/CLOCKS_PER_SEC);
	
	//free memory
	cudaFree(obj);
	cudaFree(curvature);
	free(voxel);
}
