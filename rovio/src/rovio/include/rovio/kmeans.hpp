#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <math.h>

using namespace std;

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


void sort_centers_and_deviations(float* centers, float* deviations, int K){
    // Sort in ascending order
    float centers_copy[K];
    float deviations_copy[K];
    for (int i = 0; i < K; i++){
        centers_copy[i] = centers[i];
        deviations_copy[i] = deviations[i];
    }
    
    sort(centers_copy, centers_copy+K);

    for (int i = 0; i < K; i++){
        bool found_match = false;
        for (int j = 0; j < K; j++){
            if (centers_copy[i] == centers[j]){
                deviations[i] = deviations_copy[j];
                found_match = true;
            }
        }

        if (!found_match){
            cout << "Bro something is wrong" << std::endl;
        }
    }
    for (int i = 0; i < K; i++){
        centers[i] = centers_copy[i];
    }
}


void kmeans(vector<float> &A, float* centers, float* deviations, int K){
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
}

#endif /* KMEANS_HPP_ */