#ifndef KMEANS_HPP_
#define KMEANS_HPP_

#include <math.h>

using namespace std;

int findClusterID(vector<float> centers, int value){
    int count = 0;
    int best_center = -1;
    float best_dist = INFINITY;

    for (auto item: centers){
        float dist = abs(value - item);
        if (dist < best_dist){
            best_dist = dist;
            best_center = count;
        }
        count ++;
    }
    return best_center;
}

float kmeans(vector<float> &A, int K){
    vector<float> centers;
    unordered_set<int> dedup;

    while(centers.size() < K){

        int rid = rand()%A.size();

        if(dedup.find(A[rid]) == dedup.end()){
            centers.push_back(A[rid]);
            dedup.insert(A[rid]);
        }

    }
    dedup.clear();

    vector<int> cluster(A.size(),-1);
    float lastErr = 0;

    while(true){

        //assigning to cluster
        for(int i = 0; i<A.size(); i++){
            int cid = findClusterID(centers, A[i]);
            cluster[i] = cid;
        }

        //recalculate centers per cluster
        vector<int> cnt(K, 0);
        vector<int> sum(K, 0);
        float err=0;

        for(int i = 0; i<A.size(); i++){
            int cid = cluster[i];
            cnt[cid]++;
            sum[cid]+=A[i];

            //error
            err+=abs(static_cast<float>(A[i])-centers[cid]);
        }

        float delta = abs(lastErr - err);

        cout << "error " << err << " delta " << delta << endl;

        if(delta < 0.1)break;

        lastErr = err;

        //assign new centers

        for(int i =0; i<K; i++){
            centers[i] = (static_cast<float>(sum[i])/cnt[i]);
        }

    }

    
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

    return centers[0];
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