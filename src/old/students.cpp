
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <database_interface/db_class.h>

// all database classes must inherit from database_interface::DBClass
class keyframe : public database_interface::DBClass
{

//fields are made public in this toy example for easy access, but you
//can treat them as you would any member data of your C++ classes
public:

  //key requirement: all fields that are to be stored in the 
  //database must be wrapped as database_interface::DBField<>, 
  //templated on the type of data they hold
  
  database_interface::DBField<int> keyframe_id_;

  database_interface::DBField<std::vector<cv::Point2d> > features_;

  database_interface::DBField<std::vector<double> > keyframe_descriptors_;

  database_interface::DBField<std::string> file_location_;
  
  //key requirement: all fields must be initialized in the constructor
  //a field constructor takes the following arguments:
  // - the type of serialization used (TEXT for all fields in this toy example)
  // - the owner of the field ( usually "this", or the instance of the DBClass 
  //   that owns that field)
  // - the name of the table column corresponding to that field
  // - the name of the table in which the field is stored
  // - whether it is allowed to modify en entry in the database using a reference 
  //   to this field
  keyframe() : 
    keyframe_id_(database_interface::DBFieldBase::TEXT, 
		this, "keyframe_id", "Keyframe", true),
    features_(database_interface::DBFieldBase::TEXT, 
			this, "Features", "Keyframe", true),
    keyframe_descriptors_(database_interface::DBFieldBase::TEXT, 
		    this, "keyframe_descriptors", "Keyframe", true),
    file_location_(database_interface::DBFieldBase::TEXT, 
		 this, "file_location", "Keyframe", true)
  {
    //finally, all fields must be registered with the DBClass itself

    //one field MUST be a primary key
    //all instances of DBClass have a primary_key_field_ pointer, 
    //which must be set on construction
    primary_key_field_ = &keyframe_id_;

    //all other fields go into the fields_ array of the DBClass
    fields_.push_back(&keyframe_id_);
    fields_.push_back(&features_);
    fields_.push_back(&keyframe_descriptors_);
    fields_.push_back(&file_location_);

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
};

class Grade : public database_interface::DBClass
{
public:
  database_interface::DBField<int> grade_id_;
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> grade_subject_;
  database_interface::DBField<double> grade_grade_;

  Grade() :
    grade_id_(database_interface::DBFieldBase::TEXT, 
		this, "grade_id", "grade", true),
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "grade", true),
    grade_subject_(database_interface::DBFieldBase::TEXT, 
		   this, "grade_subject", "grade", true),
    grade_grade_(database_interface::DBFieldBase::TEXT, 
		 this, "grade_grade", "grade", true)  
  {
    primary_key_field_ = &grade_id_;
    fields_.push_back(&student_id_);
    fields_.push_back(&grade_subject_);
    fields_.push_back(&grade_grade_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);
  }
};

class StudentWithPhoto : public database_interface::DBClass
{
public:  
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> student_first_name_;
  database_interface::DBField<std::string> student_last_name_;
  database_interface::DBField< std::vector<std::string> > student_majors_;
  database_interface::DBField<double> student_gpa_;
  database_interface::DBField< std::vector<char> > student_photo_;
  
  StudentWithPhoto() : 
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "student", true),
    student_first_name_(database_interface::DBFieldBase::TEXT, 
			this, "student_first_name", "student", true),
    student_last_name_(database_interface::DBFieldBase::TEXT, 
		       this, "student_last_name", "student", true),
    student_majors_(database_interface::DBFieldBase::TEXT, 
		    this, "student_majors", "student", true),
    student_gpa_(database_interface::DBFieldBase::TEXT, 
		 this, "student_gpa", "student", true),
    student_photo_(database_interface::DBFieldBase::BINARY, 
		 this, "student_photo", "student", true)
  {
    primary_key_field_ = &student_id_;

    fields_.push_back(&student_first_name_);
    fields_.push_back(&student_last_name_);
    fields_.push_back(&student_majors_);
    fields_.push_back(&student_gpa_);
    fields_.push_back(&student_photo_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    student_photo_.setReadFromDatabase(false);
    student_photo_.setWriteToDatabase(false);
  }
};


class GradeWithSequence : public database_interface::DBClass
{
public:
  database_interface::DBField<int> grade_id_;
  database_interface::DBField<int> student_id_;
  database_interface::DBField<std::string> grade_subject_;
  database_interface::DBField<double> grade_grade_;

  GradeWithSequence() :
    grade_id_(database_interface::DBFieldBase::TEXT, 
		this, "grade_id", "grade", true),
    student_id_(database_interface::DBFieldBase::TEXT, 
		this, "student_id", "grade", true),
    grade_subject_(database_interface::DBFieldBase::TEXT, 
		   this, "grade_subject", "grade", true),
    grade_grade_(database_interface::DBFieldBase::TEXT, 
		 this, "grade_grade", "grade", true)  
  {
    primary_key_field_ = &grade_id_;
    fields_.push_back(&student_id_);
    fields_.push_back(&grade_subject_);
    fields_.push_back(&grade_grade_);

    setAllFieldsReadFromDatabase(true);
    setAllFieldsWriteToDatabase(true);

    grade_id_.setSequenceName("grade_id_seq");
    grade_id_.setWriteToDatabase(false);
  }
};


#include <boost/shared_ptr.hpp>
#include <database_interface/postgresql_database.h>

int main(int argc, char **argv)
{
  database_interface::PostgresqlDatabase 
    database("localhost", "5432",
	     "benjamin", "hex", "Students");
  if (!database.isConnected())
  {
    std::cerr << "Database failed to connect \n";
    return -1;
  }
  std::cerr << "Database connected successfully \n";


  return 0;
}
