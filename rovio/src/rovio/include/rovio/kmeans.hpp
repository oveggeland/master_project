#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <math.h>

using namespace std;

int findClusterID(float* centers, float* deviations, float value, int K){
    int best_center = -1;
    float best_dist = INFINITY;

    for (int k=0; k<K; k++){
        std::cout << "Value: " << value << " center: " << centers[k] << " dev: " << deviations[k] << std::endl;
        float dist = abs(value - centers[k]);
        if (dist < 3*deviations[k] && dist < best_dist){
            best_dist = dist;
            best_center = k;
        }
    }
    if (best_center == -1){
        std::cout << "Value " << value << " is filtered out from kmeans" << std::endl;
    }
    return best_center;
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

    int cluster[A.size()] = {-1};
    int prev_cnt[K] = {0};
    float lastErr = 0;
    
    
    while(true){
        std::cout << "Start of loop" << std::endl;

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

        std::cout << "Checking termination criteria!" << std::endl;

        // Termination criteria!
        float delta = abs(lastErr - tot_err);
        lastErr = tot_err;

        if(delta < 0.1){
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
            std::cout << "Sum is " << sum[i] << " square error is " << tot_square_err << " count is " << cnt[i] << std::endl;
            centers[i] = (static_cast<float>(sum[i])/cnt[i]);
            std::cout << "Center is " << centers[i] << std::endl;
            deviations[i] = sqrt(static_cast<float>(tot_square_err[i])/cnt[i]);
            std::cout << "Std is " << deviations[i] << std::endl;
        }

    }

    /*
    // print out results
    for(int i = 0; i<K; i++){
        cout<<"Cluster Center "<<i<< " : "<<centers[i]<<endl;
        cout<<"Cluster Elements: ";

        for(int j=0; j<cluster.size(); j++){
            if(cluster[j]==i){
                cout<<A[j]<<" ";
            }
        }
        cout<<endl;
    }
    */

    // return centers;
}

/*
int slain() {
    vector<float> x = { 71, 73, 64, 65, 61, 70, 65, 72, 63, 67, 64};
    vector<float> y = {160, 183, 154, 168, 159, 180, 145, 210, 132, 168, 141};

    // prepare the input data and shuffle them

    vector<float> mix;
    mix.insert(mix.end(),x.begin(),x.end());

    int randPos = rand()%mix.size();
    mix.insert(mix.begin()+randPos, y.begin(),y.end());

    // Clusters
    int K = 2;
    kmeans(mix, K);

    return 0;
}
*/

#endif /* KMEANS_HPP_ */