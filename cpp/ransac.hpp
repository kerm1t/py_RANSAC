#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>

struct Point3D {
    double x, y, z;
    
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};

struct Plane {
    double a, b, c, d;
    double length;
    
    Plane(double a_, double b_, double c_, double d_) 
        : a(a_), b(b_), c(c_), d(d_) {
        length = std::max(0.1, std::sqrt(a*a + b*b + c*c));
    }
    
    double distanceToPoint(const Point3D& point) const {
        return std::abs(a * point.x + b * point.y + c * point.z + d) / length;
    }
};

class RANSAC3D {
private:
    std::vector<Point3D> pointCloud;
    std::mt19937 rng;
    
public:
    RANSAC3D() : rng(std::random_device{}()) {}    
    
    // Create plane from three points
    Plane createPlaneFromPoints(const Point3D& p1, const Point3D& p2, const Point3D& p3) {
        // Calculate plane equation ax + by + cz + d = 0
        double a = (p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y);
        double b = (p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z);
        double c = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
        double d = -(a * p1.x + b * p1.y + c * p1.z);
        
        return Plane(a, b, c, d);
    }
    
    // Run RANSAC algorithm
    std::vector<int> runRANSAC(int maxIterations = 5, double distanceThreshold = 0.5) {
        if (pointCloud.size() < 3) {
            std::cerr << "Error: Need at least 3 points for plane fitting" << std::endl;
            return {};
        }
        
        std::vector<int> bestInliers;
        std::uniform_int_distribution<int> dist(0, pointCloud.size() - 1);
        
        for (int iter = 0; iter < maxIterations; ++iter) {
            std::vector<int> inliers;
            
            // Select 3 random points
            while (inliers.size() < 3) {
                int randomIndex = dist(rng);
                // Ensure no duplicates
                if (std::find(inliers.begin(), inliers.end(), randomIndex) == inliers.end()) {
                    inliers.push_back(randomIndex);
                }
            }
            
            // Create plane from the 3 random points
            Plane plane = createPlaneFromPoints(
                pointCloud[inliers[0]], 
                pointCloud[inliers[1]], 
                pointCloud[inliers[2]]
            );
            
            // Test all other points
            for (size_t i = 0; i < pointCloud.size(); ++i) {
                // Skip if this point was used to create the plane
                if (std::find(inliers.begin(), inliers.end(), i) != inliers.end()) {
                    continue;
                }
                
                double distance = plane.distanceToPoint(pointCloud[i]);
                if (distance <= distanceThreshold) {
                    inliers.push_back(i);
                }
            }
            
            // Keep the best set of inliers (largest set)
            if (inliers.size() > bestInliers.size()) {
                bestInliers = inliers;
            }
        }
        
        return bestInliers;
    }
    
    // Get outliers (points not in inliers)
    std::vector<int> getOutliers(const std::vector<int>& inliers) {
        std::vector<int> outliers;
        std::vector<bool> isInlier(pointCloud.size(), false);
        
        // Mark inliers
        for (int idx : inliers) {
            isInlier[idx] = true;
        }
        
        // Collect outliers
        for (size_t i = 0; i < pointCloud.size(); ++i) {
            if (!isInlier[i]) {
                outliers.push_back(i);
            }
        }
        
        return outliers;
    }
    
    
    size_t getPointCount() const { return pointCloud.size(); }
};

int main() {
    RANSAC3D ransac;
    
    // Run RANSAC
    int maxIterations = 5;
    double distanceThreshold = 0.5;
    
    std::cout << "Running RANSAC with " << maxIterations 
              << " iterations and distance threshold " << distanceThreshold << std::endl;
    
    std::vector<int> inliers = ransac.runRANSAC(maxIterations, distanceThreshold);
    std::vector<int> outliers = ransac.getOutliers(inliers);
    
    // Print and save results
    ransac.printResults(inliers, outliers);
    ransac.saveResults(inliers, outliers);
    
    std::cout << "\nRANSAC plane fitting completed!" << std::endl;
    std::cout << "Results saved to inliers.txt and outliers.txt" << std::endl;
    std::cout << "You can visualize these files using plotting software like gnuplot or Python." << std::endl;
    
    return 0;
}