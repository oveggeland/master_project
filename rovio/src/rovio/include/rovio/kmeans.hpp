#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <math.h>

using namespace std;

int findClusterID(vector<float> centers, vector<float> deviations, float value){
    int count = 0;
    int best_center = -1;
    float best_dist = INFINITY;

    for (auto item: centers){
        std::cout << "Value: " << value << " center: " << item << " dev: " << deviations[count] << std::endl;
        float dist = abs(value - item);
        if (dist < deviations[count] && dist < best_dist){
            best_dist = dist;
            best_center = count;
        }
        count ++;
    }
    if (best_center == -1){
        std::cout << "Value " << value << " is filtered out from kmeans" << std::endl;
    }
    return best_center;
}

void kmeans(vector<float> &A, vector<float> &centers, vector<float> &deviations, int K){
    unordered_set<int> dedup;

    while(centers.size() < K){

        int rid = rand()%A.size();

        if(dedup.find(A[rid]) == dedup.end()){
            centers.push_back(A[rid]);
            deviations.push_back(INFINITY);     
            dedup.insert(A[rid]);
        }

    }
    dedup.clear();

    vector<int> cluster(A.size(),-1);
    vector<int> prev_cnt(K, 0);
    float lastErr = 0;
    
    
    while(true){
        std::cout << "Start of loop" << std::endl;

        //assigning to cluster
        for(int i = 0; i<A.size(); i++){
            int cid = findClusterID(centers, deviations, A[i]);
            cluster[i] = cid;
        }

        //recalculate centers per cluster
        vector<int> cnt(K, 0);
        vector<int> sum(K, 0);
        vector<int> tot_square_err(K, 0);
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
        if(delta < 0.1){
            bool done = true;
            for (int k = 0; k<K; k++){
                if (cnt[k] != prev_cnt[k]){
                    done = false;
                }
            }
            if (done){
                break;
            }

        };


        lastErr = tot_err;
        prev_cnt = cnt;

        //assign new centers

        for(int i =0; i<K; i++){
            centers[i] = (static_cast<float>(sum[i])/cnt[i]);
            deviations[i] = sqrt(static_cast<float>(tot_square_err[i])/cnt[i]);
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