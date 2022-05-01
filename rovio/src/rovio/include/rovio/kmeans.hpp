#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <math.h>

using namespace std;

// For standard kmeans algorithm
float kmeans(vector<float> &A, float* centers, float* deviations, int K);
int findClusterID(float* centers, float* deviations, float value, int K);

// Helper to sort initial values of centers and deviations. Gives consistency in visualization outside of the algorithm
void sort_centers_and_deviations(float* centers, float* deviations, int K);

// Helpers to calculate silhoutte score
float calculate_b(vector<float> A, int*clusters, float*centers, int K, int i);
float calculate_a(vector<float> A, int*clusters, int i);

void copy_array(float* src, float* dst, int size);


// Entry point. Run kmeans for different amounts of K, choose best silhouette score
int cluster_algorithm(vector<float> &A, float* centers, float* deviations, int K_max){
    float best_score = -1;
    float center_copy[K_max];
    float dev_copy[K_max];

    copy_array(centers, center_copy, K_max);
    copy_array(deviations, dev_copy, K_max);
    std::cout << "Centers copy is now " << center_copy[0] << " " << center_copy[1] << std::endl;

    for (int k=1; k<K_max; k++){
        float score = kmeans(A, centers, deviations, k);
        std::cout << "Score with " << k << " clusters is: " << score << std::endl;
        std::cout << "Centers is now " << centers[0] << " " << centers[1] << std::endl;
        if (score > best_score){
            best_score = score;
            copy_array(centers, center_copy, K_max);
            copy_array(deviations, dev_copy, K_max);
        }
        else{
            copy_array(center_copy, centers, K_max);
            copy_array(dev_copy, deviations, K_max);
            std::cout << "Centers is now " << centers[0] << " " << centers[1] << std::endl;
            return k;
        }
    }
    // Default: should not get here? Ignore clustering
    return 0;
}


// kmeans algorithm
float kmeans(vector<float> &A, float* centers, float* deviations, int K){
    unordered_set<int> dedup;

    int count = 0;
    while(count < K){

        int rid = rand()%A.size();

        if(dedup.find(A[rid]) == dedup.end()){
            centers[count] = A[rid];
            deviations[count] = INFINITY;     
            dedup.insert(A[rid]);
            count ++;
        }
    }
    dedup.clear();
    sort_centers_and_deviations(centers, deviations, K);

    int cluster[A.size()] = {-1};
    int prev_cnt[K] = {0};
    float lastErr = 0;
    
    
    while(true){
        //assigning to cluster
        for(int i = 0; i<A.size(); i++){
            int cid = findClusterID(centers, deviations, A[i], K);
            cluster[i] = cid;
        }

        //recalculate centers per cluster
        int cnt[K] = {0};
        float sum[K] = {0};
        float tot_square_err[K] = {0};
        float err=0;
        float tot_err=0;
        

        for(int i = 0; i<A.size(); i++){
            int cid = cluster[i];
            if (cid != -1){
                cnt[cid]++;
                sum[cid]+=A[i];

                //error
                err=abs(static_cast<float>(A[i])-centers[cid]);
                tot_err += err;
                tot_square_err[cid] += pow(err, 2);
            }
        }


        // Termination criteria!
        float delta = abs(lastErr - tot_err);
        lastErr = tot_err;

        if(delta < 0.1){
            break; // Comment out this if you want to use the filtering
            bool done = true;
            for (int k = 0; k<K; k++){
                if (cnt[k] != prev_cnt[k]){
                    done = false;
                }
                prev_cnt[k] = cnt[k];
            }
            if (done){
                break;
            }

        };


        //assign new centers

        for(int i =0; i<K; i++){
            centers[i] = (static_cast<float>(sum[i])/cnt[i]);
            deviations[i] = sqrt(static_cast<float>(tot_square_err[i])/cnt[i]);
        }
    }

    // Perform silhoutte calculations
    if (K == 0){

    }
    float sum = 0;

    for (int i = 0; i < A.size(); i++){
        // calculate a and b value for each point i
        float a_val = calculate_a(A, cluster, i);
        float b_val = 0;
        if (K >= 2){
            b_val = calculate_b(A, cluster, centers, K, i);
        }

        std::cout << "a_val is " << a_val << " b_val is " << b_val << std::endl;
        if (a_val != 0){
            sum += (b_val-a_val)/max(b_val, a_val);
        }
    }

    std::cout << "sum is " << sum << " A.size is " << A.size() << std::endl;
    return sum / A.size();
}


// Helper, find closest cluster
int findClusterID(float* centers, float* deviations, float value, int K){
    int best_center = -1;
    float best_dist = INFINITY;

    for (int k=0; k<K; k++){
        float dist = abs(value - centers[k]);
        if (dist < 3*deviations[k] && dist < best_dist){
            best_dist = dist;
            best_center = k;
        }
    }
    return best_center;
}


// Helper, calculate a value in silhouette method
float calculate_a(vector<float> A, int*clusters, int i){
    float val = A[i];
    float sum = 0;
    int count = -1;
    for (int j = 0; j < A.size(); j++){
        if (clusters[i] == clusters[j]){
            sum += abs(A[i] - A[j]);
            count++;
        }
    }
    if (count <= 0){
        return 0;
    }
    return sum / count;
}


// Helper, calculate b value in silhouette method
float calculate_b(vector<float> A, int*clusters, float*centers, int K, int i){
    // Find closest center
    int my_center = clusters[i];
    float best_dist = INFINITY;
    int closest_center = -1;
    for (int k = 0; k < K; k++){
        if (k != my_center){
            float dist = abs(centers[k] - centers[my_center]);
            if (dist < best_dist && centers[k] != -1){
                best_dist = dist;
                closest_center = k;
            }
        }
    }

    // Calculate average distance to this center
    float sum = 0;
    int count = 0;
    for (int i = 0; i < A.size(); i++){
        if (clusters[i] == closest_center){
            sum += clusters[i];
            count++;
        }
    }
    if (count <= 0){
        return 0;
    }
    return sum / count;
}


void sort_centers_and_deviations(float* centers, float* deviations, int K){
    // Sort in ascending order
    float centers_copy[K];
    float deviations_copy[K];

    copy_array(centers, centers_copy, K);
    copy_array(deviations, deviations_copy, K);
    
    sort(centers_copy, centers_copy+K);

    for (int i = 0; i < K; i++){
        bool found_match = false;
        for (int j = 0; j < K; j++){
            if (centers_copy[i] == centers[j]){
                deviations[i] = deviations_copy[j];
                found_match = true;
            }
        }
    }
    copy_array(centers_copy, centers, K);
}

void copy_array(float* src, float* dst, int size){
    for (int i = 0; i < size; i ++){
        dst[i] = src[i];
    }
}

#endif /* KMEANS_HPP_ */