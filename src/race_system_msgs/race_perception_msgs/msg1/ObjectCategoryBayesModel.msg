#### Object Category

###msg: ObjectCategory.msg
###Key: OC_<CatName>_<CatID>
###Topic name: not used

###Value:
Header header   ## ROS msg header
string cat_name    ##object category name (do not include _ in the cat name)
uint32 cat_id  ##Cat ID generated by Conceptualizer
string[] rtov_keys  #list keys of rep. of tracked object view (RTOI)
float64 icd   #intra-category distance

#### Naive Bayes Object Category Model (Hamidreza)
float32[] likelihood        # all object representation likelihood 
float64 prior_probability   # prior category probability


