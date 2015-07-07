#include <string>
#include <vector>
#include <sys/stat.h>

#include <database_interface/db_class.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>
#include <stdexcept>

// I can't figure out how to separate this into a .h file and a .cpp file and have CMake
// play nicely, so I'm just going to implement it all 

using namespace cv;
using namespace std;

// all database classes must inherit from database_interface::DBClass
class keyframe : public database_interface::DBClass
{

//fields are made public in this toy example for easy access, but you
//can treat them as you would any member data of your C++ classes
private:

  database_interface::DBField<int> keyframe_id_;

  database_interface::DBField< std::vector<double> > keyframe_descriptors_;

  database_interface::DBField<std::string> file_location_;

  database_interface::DBField<std::vector<double> > keyframe_feature_x_;
  
  database_interface::DBField<std::vector<double> > keyframe_feature_y_;
  
public:
  //key requirement: all fields that are to be stored in the 
  //database must be wrapped as database_interface::DBField<>, 
  //templated on the type of data they hold
  
  //key requirement: all fields must be initialized in the constructor
  //a field constructor takes the following arguments:
  // - the type of serialization used (TEXT for all fields in this toy example)
  // - the owner of the field ( usually "this", or the instance of the DBClass 
  //   that owns that field)
  // - the name of the table column corresponding to that field
  // - the name of the table in which the field is stored
  // - whether it is allowed to modify en entry in the database using a reference 
  //   to this field
  
  // Default constructor, used for retreiving values from the database into
  // a keyframe object.
    keyframe( ) :
	// Descriptors is a 64xn Mat, raw_keypoints_kf is a n-long vector.     
    keyframe_id_(database_interface::DBFieldBase::TEXT, 
		this, "keyframe_id", "keyframe", true),
    keyframe_descriptors_(database_interface::DBFieldBase::TEXT, 
		    this, "keyframe_descriptors", "keyframe", true),
    file_location_(database_interface::DBFieldBase::TEXT, 
		 this, "file_location", "keyframe", true),
    keyframe_feature_x_(database_interface::DBFieldBase::TEXT, 
			this, "keyframe_feature_x", "keyframe", true),
    keyframe_feature_y_(database_interface::DBFieldBase::TEXT, 
			this, "keyframe_feature_y", "keyframe", true)
  {
  
    primary_key_field_ = &keyframe_id_;

    fields_.push_back(&keyframe_descriptors_);
    fields_.push_back(&file_location_);
    fields_.push_back(&keyframe_feature_x_);
    fields_.push_back(&keyframe_feature_y_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
  }
  
  keyframe( int keyframe_id, std::vector<cv::KeyPoint> raw_keypoints_kf,
  	std::string fileLocation, cv::Mat kf_descriptors) :
	// Descriptors is a 64xn Mat, raw_keypoints_kf is a n-long vector.     
    keyframe_id_(database_interface::DBFieldBase::TEXT, 
		this, "keyframe_id", "keyframe", true),
    keyframe_descriptors_(database_interface::DBFieldBase::TEXT, 
		    this, "keyframe_descriptors", "keyframe", true),
    file_location_(database_interface::DBFieldBase::TEXT, 
		 this, "file_location", "keyframe", true),
    keyframe_feature_x_(database_interface::DBFieldBase::TEXT, 
			this, "keyframe_feature_x", "keyframe", true),
    keyframe_feature_y_(database_interface::DBFieldBase::TEXT, 
			this, "keyframe_feature_y", "keyframe", true)
  {
  
  	keyframe_id_.data() = keyframe_id;
	file_location_.data() = fileLocation;
	for (int i = 0; i < raw_keypoints_kf.size(); i++){
		keyframe_feature_x_.data().push_back(raw_keypoints_kf[i].pt.x);
		keyframe_feature_y_.data().push_back(raw_keypoints_kf[i].pt.y);
		for (int j = 0; j < 64; j++){
			keyframe_descriptors_.data().push_back(kf_descriptors.at<float>(i,j));
		}
	}
    //finally, all fields must be registered with the DBClass itself

    //one field MUST be a primary key
    //all instances of DBClass have a primary_key_field_ pointer, 
    //which must be set on construction
    
    primary_key_field_ = &keyframe_id_;

    //all other fields go into the fields_ array of the DBClass
    fields_.push_back(&keyframe_descriptors_);
    fields_.push_back(&file_location_);
    fields_.push_back(&keyframe_feature_x_);
    fields_.push_back(&keyframe_feature_y_);

    //optional: let all fields be read automatically when an instance 
    //of a student is retrieved from the database
    setAllFieldsReadFromDatabase(true);
    //optional: let all fields be written automatically when an instance 
    //of a student is saved the database
    setAllFieldsWriteToDatabase(true);
    //(these options are usful if you have a very large field (e.g. a 
    // binary bitmap with the picture of the student) which you do not 
    //want retrieved automatically whenever you get a student info 
    //from the database
  }
  
	void getKFData(int &keyframe_id, std::vector<cv::KeyPoint> &raw_keypoints_kf,
		std::string &fileLocation, cv::Mat &kf_descriptors){
		keyframe_id = keyframe_id_.data();
		
		// Can't assume that anything passed in is empty, so...
		
		raw_keypoints_kf.erase(raw_keypoints_kf.begin(), raw_keypoints_kf.end() );
		
		fileLocation = file_location_.data();
		kf_descriptors = cv::Mat::zeros(keyframe_feature_x_.data().size(), 64, CV_64FC1);
		
		for (int i = 0; i < keyframe_feature_x_.data().size(); i++){
			raw_keypoints_kf.push_back(cv::KeyPoint(keyframe_feature_x_.data().at(i) ,
				keyframe_feature_y_.data().at(i) , 0) );
			int mult = i * 64;
			for (int j = 0; j < 64; j++){
				kf_descriptors.at<float>(i,j) = keyframe_descriptors_.data().at(mult + j);
			}
		}
	}

};

class keyframeDatabase
{
	public:
	keyframeDatabase(std::string DBName){
	
		if (ros::param::has("dbUsername") ){
			ros::param::get("dbUsername", username);
		}else{
			username = "benjamin";
		}
		
		if (ros::param::has("dbPassword") ){
			ros::param::get("dbPassword", password);
		}else{
			password = "hex";
		}
		
		database = new database_interface::PostgresqlDatabase(
			"localhost", "5432", username, password, DBName);
		if (!database->isConnected())
		{
			//std::cerr << "Database failed to connect \n";
			throw std::invalid_argument( "Database failed to connect" );
		}
		std::cerr << "Database connected successfully \n";
		
		keyframe_id = 0;
	}
	
	bool addKeyframe(std::vector<cv::KeyPoint> raw_keypoints_kf,
  		std::string fileLocation, cv::Mat kf_descriptors){
  		
		keyframe newElement(keyframe_id, raw_keypoints_kf, fileLocation, kf_descriptors);
		
		std::vector< boost::shared_ptr<keyframe> > kf;
		char where_clause[20];
		sprintf(where_clause, "keyframe_id=%d", keyframe_id);
		
		if(!database->getList(kf, where_clause) ){
			ROS_ERROR("Problem connecting to database.\n");
		//	res.success.data = false;
			return false;
		}else{
			if (kf.size() == 0){
				// We're good to go. Add on. 
			
				if (!database->insertIntoDatabase(&newElement) ){
					std::cerr << "Insertion failed!\n";
					//res.success.data = false;
					return false;
				}
				keyframe_id_stack.push_back(keyframe_id);
				keyframe_id++;  // Increment the keyframe id only with successful inclusion. 

ROS_INFO("Keyframe id insert is %d", keyframe_id);
			//	res.success.data = true;
				return true;
			}else{
				ROS_ERROR("Somehow inserting on the wrong key.\n");
			//	res.success.data = false;
				return false;
			}
		}
		
		// Shouldn't have issue with duplicate keyframe id's, but... TODO check that
		
  	}
  	
  	bool getKeyframe(int &kid, std::vector<cv::KeyPoint> &raw_keypoints_kf,
		std::string &fileLocation, cv::Mat &kf_descriptors){
		
		if (keyframe_id_stack.size() == 0){
			// No more keyframes
			ROS_INFO("Keyframes are empty.");
			return false;
		}
		kid = keyframe_id_stack.back(); // Get the last keyframe. I'm not deleting keyframes because 
				// I want to keep the database intact for offline work. A few extra images won't hurt.
					
		ROS_INFO("KID is %d", kid);
					
		std::vector< boost::shared_ptr<keyframe> > kf;
		char where_clause[20];
		sprintf(where_clause, "keyframe_id=%d", kid);
		//	std::string where_clause = "keyframe_id=3";
		if(!database->getList(kf, where_clause) ){
			std::cerr << "Problem connecting to database.\n";
		//	res.success.data = false;
			return false;
		}else{
cout << kf.size() << " is the size " << endl;
			if (kf.size() == 0){
				std::cerr << "No entries with this key in getKeyframe.\n";
			//	res.success.data = false;
				return false;
			}
			else{ // Definitely has one entry, since we're selecting on keyframe_id, which is the primary key. 
	 			kf[0]->getKFData(kid, raw_keypoints_kf, fileLocation, kf_descriptors);
				if (file_exists(fileLocation) ){
					keyframe_id_stack.pop_back(); // Only take it off the stack if it was retrieved.
					return true;
				}else{
				//	res.success.data = false;
					ROS_INFO("File does not exist at %s", fileLocation.c_str() );
					return false;
				}
			}
		}
	}
	
	inline bool file_exists (const std::string& name) {
		struct stat buffer;   
		return (stat (name.c_str(), &buffer) == 0);
	}
	
	private:
		std::string username;
		std::string password;
		std::string DBName;
		database_interface::PostgresqlDatabase* database;
		int keyframe_id;
		std::vector<int> keyframe_id_stack;
		
};
/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "Database_client");  
  ros::NodeHandle n;
	keyframeDatabase db;
	
	  Mat img = imread("/home/benjamin/Research/Code/ESM_lib/res/im000.pgm");
  

	FastFeatureDetector detector( 50 ); 
	SurfDescriptorExtractor extractor;  
	vector<cv::KeyPoint> raw_kps_keyframe;
	Mat descriptors_keyframe;
	detector.detect(img, raw_kps_keyframe);
	extractor.compute(img, raw_kps_keyframe, descriptors_keyframe);
	
	ros::Time begin = ros::Time::now();
	
	if(db.addKeyframe(raw_kps_keyframe, "./test.png", descriptors_keyframe) ){
		ROS_INFO("Success adding");
	}else{
		ROS_ERROR("Error adding keyframe!");
	}
	imwrite("./test.png", img);

	ros::Time end = ros::Time::now();

	ros::Duration passed = end - begin;
	cout << passed.toSec() << endl;
	
	
begin = ros::Time::now();
	int kid;
	vector<KeyPoint> rkkf;
	string location;
	Mat desc;
	if (db.getKeyframe(kid, rkkf, location, desc) ){
		ROS_INFO("Got the info back");
	}else{
		ROS_ERROR("Had a snafu...");
	}
	
	end = ros::Time::now();
	passed = end-begin;
	cout << passed.toSec() << endl;
	

	return 0;
}*/
