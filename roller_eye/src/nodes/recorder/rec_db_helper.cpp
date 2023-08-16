#include "ros/ros.h"
#include "plog_plus.h"
#include "plterrno.h"
#include "plt_tools.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include "roller_eye/recorder_mgr.h"
#include"plt_assert.h"


namespace roller_eye {


RecDBHelper::RecDBHelper()
{
  load();
}

RecDBHelper::~RecDBHelper()
{
  close();
}

void RecDBHelper::load(){
    if (0 != access(REC_FILES_DIR.c_str(), F_OK)) {
        if (0 != mkdir(REC_FILES_DIR.c_str(), 0755)) {
            perror("mkdir error");
            PLOG_ERROR(REC_TAG, "RecDBHelper mkdir error: %s", REC_FILES_DIR.c_str());
        }
    }

    bool need_restore = false;
    if (0 != access(DB_NAME.c_str(), F_OK)) {
        int num = getFileNumByType(REC_FILES_DIR.c_str(), "-");
        if (num > 0) {
            PLOG_WARN(REC_TAG, "DB file not found, but media file num : %d", num);
            unlink(DB_NAME.c_str());
            need_restore = true;
        }
    }

    plt_assert(0 == openDB(DB_NAME, &m_db));
    createTable(DB_REC_TABLE_NAME);
    // printTable(DB_REC_TABLE_NAME);

    std::string sql;
    sql = "INSERT INTO " + DB_REC_TABLE_NAME + \
        " VALUES(@_file_id, @_path, @_duration, @_type, @_size, @_create_time);";
    int rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtInsert, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 insert: %s", sqlite3_errmsg(m_db));
    }

    sql = "SELECT path FROM " + DB_REC_TABLE_NAME + " WHERE file_id=@_file_id;";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtQueryPath, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 query1: %s", sqlite3_errmsg(m_db));
    }

    sql = "SELECT COUNT(*) FROM " + DB_REC_TABLE_NAME + " WHERE type=@_type;";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtQueryTypeNum, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 query2: %s", sqlite3_errmsg(m_db));
    }

    sql = "SELECT COUNT(*) FROM " + DB_REC_TABLE_NAME + " WHERE type=@_type1 OR type=@_type2;";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtQueryAllNum, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 query2: %s", sqlite3_errmsg(m_db));
    }

    sql = "SELECT file_id, path, duration, type, size, create_time FROM " + DB_REC_TABLE_NAME + \
        " WHERE type=@_type ORDER BY create_time DESC LIMIT @_start, @_size;";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtQueryTypeList, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 query3: %s", sqlite3_errmsg(m_db));
    }

    sql = "SELECT file_id, path, duration, type, size, create_time FROM " + DB_REC_TABLE_NAME + \
        " WHERE type=@_type1 OR type=@_type2 ORDER BY create_time DESC LIMIT @_start, @_size;";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtQueryAllList, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 query4: %s", sqlite3_errmsg(m_db));
    }

    sql = "DELETE FROM " + DB_REC_TABLE_NAME + " WHERE file_id=@_file_id";
    rc = sqlite3_prepare_v2(m_db, sql.c_str(), -1, &m_stmtDelete, NULL);
    if ( rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG, "sqlite3_prepare_v2 delete: %s", sqlite3_errmsg(m_db));
    }

    if (need_restore) {
        restoreDB(REC_FILES_DIR.c_str());
    }
}

int RecDBHelper::tableExistCB(void *data, int cNum, char *column_values[], char *column_names[])
{
	if ( 1 == cNum ) {
		int tnum = atoi(*(column_values));
		if (data != nullptr) {
			*(int*)data = tnum;
		}
	}
	return 0;
}

bool RecDBHelper::isTableExist(const std::string& tableName)
{
	std::string strFindTable = "SELECT COUNT(*) FROM sqlite_master WHERE type ='table' and name ='" + tableName + "'";
	char* sErrMsg = nullptr;

	int nTableNums = 0;
	if (sqlite3_exec(m_db, strFindTable.c_str(), &tableExistCB, &nTableNums, &sErrMsg) != SQLITE_OK) {
		PLOG_ERROR(REC_TAG, "sqlite3_exec error: %s\n", sErrMsg);
        return false;
	}

	return nTableNums > 0;
}

void RecDBHelper::restoreDB(const char* files_dir)
{
    PLOG_INFO(REC_TAG, "restoreDB start...");
    DIR* dir = opendir(files_dir);
    if (dir == NULL) {
        perror("opendir:");
        exit(1);
    }
    int totalPic = 0;
    int totalThumb = 0;
    int totalMp4 = 0;
    int typeInDB = -1;
    int dur = -1;
    string fullPath, nameString, fileID;
    struct dirent* dirObj = NULL;

    while ((dirObj = readdir(dir)) != NULL) {
        if (strcmp(dirObj->d_name,".")==0 || strcmp(dirObj->d_name,"..")==0) {
            continue;
        }
        if (dirObj->d_type == DT_DIR) {
            PLOG_WARN(REC_TAG, "Found dir: %s", dirObj->d_name);
            continue;
        }
        if (dirObj->d_type == DT_REG) {
            nameString = string(dirObj->d_name);
            fullPath = files_dir + nameString;
            fileID = nameString.substr(nameString.find("_") + 1, FILE_ID_LEN);
            //EX. 2019-11-23-14-25-38_bfct94Hj2534fPv3-1.jpg
            if (nameString.rfind("-" + to_string(record::RECORD_TYPE_SNAPSHOT) + ".jpg") != string::npos) {
                typeInDB = record::RECORD_TYPE_SNAPSHOT;
                totalPic++;
            }
            //EX. 2019-11-23-14-23-13_Sv2AZ9aZs6m2I682-3.jpg
            else if (nameString.rfind("-" + to_string(record::RECORD_TYPE_THUMB) + ".jpg") != string::npos) {
                typeInDB = record::RECORD_TYPE_THUMB;
                totalThumb++;
            }
            //EX. 2019-11-23-14-23-13_567h5fs9op60Bxj2_Sv2AZ9aZs6m2I682-14870.mp4
            else if (nameString.rfind(".mp4") != string::npos) {
                typeInDB = record::RECORD_TYPE_RECORD;
                dur = RecorderMP4::getFileDuration(fullPath.c_str());
                totalMp4++;
            }
            else {
                PLOG_WARN(REC_TAG, "Skip file: %s", dirObj->d_name);
                continue;
            }

            struct stat statbuf;
            if (0 != stat(fullPath.c_str(), &statbuf)) {
                PLOG_ERROR(REC_TAG, "stat file error: %s", dirObj->d_name);
                continue;
            }

            int rc = this->insertItem(fileID,
                fullPath,
                dur,
                typeInDB,
                statbuf.st_size,
                statbuf.st_mtim.tv_sec);
            if (rc != 0) {
                PLOG_ERROR(REC_TAG, "INSERT DB ERROR CODE: %d", rc);
            }
        }

    } //while
    PLOG_INFO(REC_TAG, "restoreDB end. total: %d\n mp4: %d, thumb: %d, picture: %d.",
        totalMp4 + totalThumb + totalPic, totalMp4, totalThumb, totalPic);
}

int RecDBHelper::openDB(const std::string& dbName, sqlite3** m_db)
{
    int ret = sqlite3_open(dbName.c_str(), m_db);
	if (ret != SQLITE_OK) {
		PLOG_ERROR(REC_TAG, "%s : %s\n", sqlite3_errmsg(*m_db), dbName.c_str());
        return -1;
	}
    return 0;
}

void RecDBHelper::createTable(const std::string& tableName)
{
    int rc;
    sqlite3_stmt *pStmt;
    std::string sql = "CREATE TABLE  IF NOT EXISTS " + tableName + "(" \
        "file_id        CHAR(" + to_string(FILE_ID_LEN) + ") PRIMARY KEY," \
        "path           TEXT NOT NULL," \
        "duration       INTEGER NOT NULL," \
        "type           INTEGER NOT NULL," \
        "size           INTEGER NOT NULL," \
        "create_time    INTEGER NOT NULL" \
        ");";

    rc = sqlite3_prepare_v2(m_db, sql.c_str(), sql.length(), &pStmt, NULL);
    if (rc != SQLITE_OK) {
		perror("createTable sqlite3_prepare_v2 error:");
        goto out;
	}
    rc = sqlite3_step(pStmt);
    if (rc == SQLITE_DONE) {
        PLOG_INFO(REC_TAG, "created table: %s\n", tableName.c_str());
    } else {
        PLOG_ERROR(REC_TAG, "create table error: %d \n", rc);
    }
out:
    sqlite3_finalize(pStmt);
}

void RecDBHelper::printTable(const std::string& tableName)
{
    int rc = -1;
    int nrow = 0;
    int ncolumn = 0;
    char **azResult;
    std::string sql;
    char *zErrMsg = NULL;

    sql = "SELECT * FROM " + tableName + ";";
    rc = sqlite3_get_table(m_db, sql.c_str(), &azResult, &nrow, &ncolumn, &zErrMsg);
    if (rc != SQLITE_OK) {
        PLOG_ERROR(REC_TAG,  "Can't get table: %s", sqlite3_errmsg(m_db));
        return;
    }

    PLOG_INFO(REC_TAG,"nrow = %d, ncolumn = %d\n", nrow, ncolumn );
    if( nrow !=0 && ncolumn != 0 ) {//有查询结果,不包含表头所占行数
        int nIndex = ncolumn;
        int i=0,j=0;
        for (j=0; j<ncolumn; j++) {
            printf("%s\t", azResult[j]);
        }
        printf("\n");

        for(i=0; i<nrow; i++) {
            for(j=0; j<ncolumn; j++) {
                printf("%s\t", azResult[nIndex++]);
            }
            printf("\n");
        }
    }

	sqlite3_free_table(azResult);
}

int RecDBHelper::insertItem(const string& fileId, const string& path, int duration, int type, int64_t size, int64_t create_time)
{
    sqlite3_stmt *stmt = m_stmtInsert;
    int rc = -1;
    int idx = -1;
    idx = sqlite3_bind_parameter_index( stmt, "@_file_id" );
    rc = sqlite3_bind_text( stmt, idx, fileId.c_str(), -1, SQLITE_STATIC );
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-1--result code: %d \n", rc);
        return rc;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_path" );
    rc = sqlite3_bind_text( stmt, idx, path.c_str(), -1, SQLITE_STATIC );
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-2--result code: %d \n", rc);
        return rc;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_duration" );
    rc = sqlite3_bind_int( stmt, idx, duration);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-3--result code: %d \n", rc);
        return rc;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_type" );
    rc = sqlite3_bind_int( stmt, idx, type);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-4--result code: %d \n", rc);
        return rc;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_size" );
    rc = sqlite3_bind_int64( stmt, idx, (sqlite3_int64)size);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-5--result code: %d \n", rc);
        return rc;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_create_time" );
    rc = sqlite3_bind_int64( stmt, idx, (sqlite3_int64)create_time);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-6--result code: %d \n", rc);
        return rc;
    }
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        PLOG_ERROR(REC_TAG,"---result code: [%d] %s\n", rc, sqlite3_errmsg(m_db));
        sqlite3_reset(stmt);
        return rc;
    }

    sqlite3_reset(stmt);
    return SQLITE_OK;
}

bool RecDBHelper::deleteItem(const string& fileId)
{
    sqlite3_stmt *stmt = m_stmtDelete;
    int idx = sqlite3_bind_parameter_index( stmt, "@_file_id" );
    int rc = sqlite3_bind_text( stmt, idx, fileId.c_str(), -1, SQLITE_STATIC );
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-1--result code: %d \n", rc);
        return false;
    }

    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        PLOG_ERROR(REC_TAG,"---result code: [%d] %s\n", rc, sqlite3_errmsg(m_db));
        sqlite3_reset(stmt);
        return false;
    }

    sqlite3_reset(stmt);
    return true;
}

string RecDBHelper::queryFilePath(const string& fileId)
{
    sqlite3_stmt *stmt = m_stmtQueryPath;
    std::string path;
    int idx = sqlite3_bind_parameter_index( stmt, "@_file_id" );
    int rc = sqlite3_bind_text( stmt, idx, fileId.c_str(), -1, SQLITE_STATIC );
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-1--result code: %d \n", rc);
        return path;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        path = (const char*)sqlite3_column_text(stmt, 0);
        std::clog << "file path = " << path << std::endl;
    }

    sqlite3_reset(stmt);
    return path;
}

int RecDBHelper::queryTypeNum(int type)
{
    int num = -1;
    int idx, rc;
    sqlite3_stmt *stmt = nullptr;
    if (type == record::RECORD_TYPE_ALL) {
        stmt = m_stmtQueryAllNum;
        idx = sqlite3_bind_parameter_index( stmt, "@_type1" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_SNAPSHOT);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-01--result code: %d \n", rc);
            return num;
        }
        idx = sqlite3_bind_parameter_index( stmt, "@_type2" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_RECORD);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-02--result code: %d \n", rc);
            return num;
        }
    } else {
        stmt = m_stmtQueryTypeNum;
        idx = sqlite3_bind_parameter_index( stmt, "@_type" );
        rc = sqlite3_bind_int( stmt, idx, type);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-4--result code: %d \n", rc);
            return num;
        }
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        num = sqlite3_column_int(stmt, 0);
        std::clog << "file num = " << num << std::endl;
    }

    sqlite3_reset(stmt);
    return num;
}

bool RecDBHelper::queryTypeList(int type, int start, int size, vector<record>& list)
{
    if (start < 0 || size <= 0) {
        PLOG_ERROR(REC_TAG,"Illegal params. start: %d, size: %d\n", start, size);
        return false;
    }
    int cnt = 0;
    int idx, rc;
    sqlite3_stmt *stmt = nullptr;
    if (type == record::RECORD_TYPE_ALL) {
        stmt = m_stmtQueryAllList;
        idx = sqlite3_bind_parameter_index( stmt, "@_type1" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_SNAPSHOT);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-01--result code: %d \n", rc);
            return false;
        }
        idx = sqlite3_bind_parameter_index( stmt, "@_type2" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_RECORD);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-02--result code: %d \n", rc);
            return false;
        }
    } else {
        stmt = m_stmtQueryTypeList;
        idx = sqlite3_bind_parameter_index( stmt, "@_type" );
        rc = sqlite3_bind_int( stmt, idx, type);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-1--result code: %d \n", rc);
            return false;
        }
    }

    idx = sqlite3_bind_parameter_index( stmt, "@_start" );
    rc = sqlite3_bind_int( stmt, idx, start);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-2--result code: %d \n", rc);
        return false;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_size" );
    rc = sqlite3_bind_int( stmt, idx, size);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-3--result code: %d \n", rc);
        return false;
    }

    record item;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        cnt++;
        //SELECT (file_id, path, duration, type, size, create_time)
        item.id = (const char*)sqlite3_column_text(stmt, 0);
        item.name = (const char*)sqlite3_column_text(stmt, 1);
        item.dur = sqlite3_column_int(stmt, 2);
        item.type = sqlite3_column_int(stmt, 3);
        item.size = sqlite3_column_int64(stmt, 4);
        item.create = (ros::Time)sqlite3_column_int64(stmt, 5);

        clog << "file name: " << item.name << ", type: " << (int)item.type << ", create: " << item.create << endl;
        list.push_back(item);
    }
    clog << "cnt: " << cnt << ", size: " << size << endl;

    sqlite3_reset(stmt);
    return true;
}

bool RecDBHelper::queryTypeList(int type, const string& startId, int size, vector<record>& list)
{
    int cnt = 0;
    int idx, rc;
    sqlite3_stmt *stmt = nullptr;
    if (type == record::RECORD_TYPE_ALL) {
        stmt = m_stmtQueryAllList;
        idx = sqlite3_bind_parameter_index( stmt, "@_type1" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_SNAPSHOT);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-01--result code: %d \n", rc);
            return false;
        }
        idx = sqlite3_bind_parameter_index( stmt, "@_type2" );
        rc = sqlite3_bind_int( stmt, idx, record::RECORD_TYPE_RECORD);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-02--result code: %d \n", rc);
            return false;
        }
    } else {
        stmt = m_stmtQueryTypeList;
        idx = sqlite3_bind_parameter_index( stmt, "@_type" );
        rc = sqlite3_bind_int( stmt, idx, type);
        if (rc != 0) {
            PLOG_ERROR(REC_TAG,"-1--result code: %d \n", rc);
            return false;
        }
    }

    idx = sqlite3_bind_parameter_index( stmt, "@_start" );
    rc = sqlite3_bind_int( stmt, idx, 0);
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-2--result code: %d \n", rc);
        return false;
    }
    idx = sqlite3_bind_parameter_index( stmt, "@_size" );
    rc = sqlite3_bind_int( stmt, idx, -1); //get all
    if (rc != 0) {
        PLOG_ERROR(REC_TAG,"-3--result code: %d \n", rc);
        return false;
    }

    bool foundId = false;
    int baseIdx = 0;
    int startIdx = -1;
    int endIdx = -1;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        if (!strcmp(startId.c_str(), (const char*)sqlite3_column_text(stmt, 0))) {
            foundId = true;
            break;
        }
        baseIdx++;
    }
    sqlite3_reset(stmt);
    if (!foundId) {
        return false;
    }

    if (size == 0) {
        size = INT_MAX - baseIdx - 1;
    }
    if (size > 0) {
        startIdx = baseIdx + 1;
        endIdx = baseIdx + size + 1;
    } else {
        startIdx = baseIdx + size;
        if (startIdx < 0)
            startIdx = 0;
        endIdx = baseIdx;
    }

    idx = 0;
    record item;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        if (idx >= startIdx && idx < endIdx) {
            //SELECT (file_id, path, duration, type, size, create_time)
            item.id = (const char*)sqlite3_column_text(stmt, 0);
            item.name = (const char*)sqlite3_column_text(stmt, 1);
            item.dur = sqlite3_column_int(stmt, 2);
            item.type = sqlite3_column_int(stmt, 3);
            item.size = sqlite3_column_int64(stmt, 4);
            item.create = (ros::Time)sqlite3_column_int64(stmt, 5);

            clog << "file name: " << item.name << ", type: " << (int)item.type << ", create: " << item.create << endl;
            list.push_back(item);
            cnt++;
        }
        idx++;
    }
    clog << "cnt: " << cnt << ", size: " << size << endl;

    sqlite3_reset(stmt);
    return true;
}

void RecDBHelper::closeAndReload(){
  close();
  load();
}

void RecDBHelper::close(){
  sqlite3_finalize(m_stmtInsert);
  sqlite3_finalize(m_stmtQueryPath);
  sqlite3_finalize(m_stmtQueryTypeNum);
  sqlite3_finalize(m_stmtQueryAllNum);
  sqlite3_finalize(m_stmtQueryTypeList);
  sqlite3_finalize(m_stmtQueryAllList);
  sqlite3_finalize(m_stmtDelete);
  sqlite3_close(m_db);
}
}