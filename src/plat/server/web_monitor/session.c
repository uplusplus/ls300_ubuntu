/*
 * rwlock.c
 *
 *  Created on: Aug 14, 2013
 *      Author: uplusplus
 */
#include <time.h>
#include "rwlock.h"
#include "session.h"

#define MAX_USER_LEN  20
#define MAX_SESSIONS 2
#define SESSION_TTL 120

#define authorize_url  "/authorize"
#define authorize_url_q  "/q=authorize&"
#define login_url      "/login.html"

// Describes web session.
struct session {
	char session_id[33]; // Session ID, must be unique
	char random[20]; // Random data used for extra user validation
	char user[MAX_USER_LEN]; // Authenticated user
	time_t expire; // Expiration timestamp, UTC
};

static struct session sessions[MAX_SESSIONS]; // Current sessions

// Protects messages, sessions, last_message_id
static pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;

static int get_qsvar(const struct mg_request_info *request_info,
		const char *name, char *dst, size_t dst_len) {
	const char *qs = request_info->query_string;
	return mg_get_var(qs, strlen(qs == NULL ? "" : qs), name, dst, dst_len);
}

static void my_strlcpy(char *dst, const char *src, size_t len) {
	strncpy(dst, src, len);
	dst[len - 1] = '\0';
}

// Get session object for the connection. Caller must hold the lock.
static struct session *get_session(const struct mg_connection *conn) {
	int i;
	const char *cookie = mg_get_header(conn, "Cookie");
	char session_id[33];
	time_t now = time(NULL);
	mg_get_cookie(cookie, "session", session_id, sizeof(session_id));
	for (i = 0; i < MAX_SESSIONS; i++) {
		if (sessions[i].expire != 0 && sessions[i].expire > now
				&& strcmp(sessions[i].session_id, session_id) == 0) {
			break;
		}
	}
	return i == MAX_SESSIONS ? NULL : &sessions[i];
}

static int strcompare(const char* str1, char* str2) {
	int i = 0;
	for (;;) {
		if (!str2[i] && !str1[i])
			return 0;
		if (str1[i] - str2[i])
			return str1[i] - str2[i];
		i++;
	}
}

// Return 1 if username/password is allowed, 0 otherwise.
static int check_password(const char *user, const char *password) {
	// In production environment we should ask an authentication system
	// to authenticate the user.
	// Here however we do trivial check that user and password are not empty
	return !strcompare(user, "admin") && !strcasecmp(password, "nimda");
	return (user[0] && password[0]);
}

// Allocate new session object
static struct session *new_session(void) {
	int i;
	time_t now = time(NULL);
	pthread_rwlock_wrlock(&rwlock);
	for (i = 0; i < MAX_SESSIONS; i++) {
		if (sessions[i].expire == 0 || sessions[i].expire < now) {
			sessions[i].expire = time(0) + SESSION_TTL;
			break;
		}
	}
	pthread_rwlock_unlock(&rwlock);
	return i == MAX_SESSIONS ? NULL : &sessions[i];
}

// Generate session ID. buf must be 33 bytes in size.
// Note that it is easy to steal session cookies by sniffing traffic.
// This is why all communication must be SSL-ed.
static void generate_session_id(char *buf, const char *random, const char *user) {
	mg_md5(buf, random, user, NULL);
}

// Redirect user to the login form. In the cookie, store the original URL
// we came from, so that after the authorization we could redirect back.
void redirect_to_login(struct mg_connection *conn,
		const struct mg_request_info *request_info) {
	mg_printf(conn, "HTTP/1.1 302 Found\r\n"
			"Set-Cookie: original_url=%s\r\n"
			"Location: %s\r\n\r\n", request_info->uri, login_url);
}

// A handler for the /authorize endpoint.
// Login page form sends user name and password to this endpoint.
void authorize(struct mg_connection *conn,
		const struct mg_request_info *request_info) {
	char user[MAX_USER_LEN], password[MAX_USER_LEN];
	struct session *session;

	// Fetch user name and password.
	get_qsvar(request_info, "user", user, sizeof(user));
	get_qsvar(request_info, "password", password, sizeof(password));

	if (check_password(user, password) && (session = new_session()) != NULL) {
		// Authentication success:
		//   1. create new session
		//   2. set session ID token in the cookie
		//   3. remove original_url from the cookie - not needed anymore
		//   4. redirect client back to the original URL
		//
		// The most secure way is to stay HTTPS all the time. However, just to
		// show the technique, we redirect to HTTP after the successful
		// authentication. The danger of doing this is that session cookie can
		// be stolen and an attacker may impersonate the user.
		// Secure application must use HTTPS all the time.
		my_strlcpy(session->user, user, sizeof(session->user));
		snprintf(session->random, sizeof(session->random), "%d", rand());
		generate_session_id(session->session_id, session->random,
				session->user);
		mg_printf(conn, "HTTP/1.1 302 Found\r\n"
				"Set-Cookie: session=%s; max-age=3600; http-only\r\n" // Session ID
				"Set-Cookie: user=%s\r\n"// Set user, needed by Javascript code
				"Set-Cookie: original_url=/; max-age=0\r\n"// Delete original_url
				"Location: /\r\n\r\n", session->session_id, session->user);
	} else {
		// Authentication failure, redirect to login.
		redirect_to_login(conn, request_info);
	}
}

// Return 1 if request is authorized, 0 otherwise.
int is_authorized(const struct mg_connection *conn,
		const struct mg_request_info *request_info) {
	struct session *session;
	char valid_id[33];
	int authorized = 0;

	// Always authorize accesses to login page and to authorize URI
	if (!strcmp(request_info->uri, login_url)
			|| !strcmp(request_info->uri, authorize_url)
			|| (request_info->query_string
					&& !strncmp(request_info->query_string, authorize_url_q,
							strlen(authorize_url_q)))) {
		return 1;
	}

	pthread_rwlock_rdlock(&rwlock);
	if ((session = get_session(conn)) != NULL) {
		generate_session_id(valid_id, session->random, session->user);
		if (strcmp(valid_id, session->session_id) == 0) {
			session->expire = time(0) + SESSION_TTL;
			authorized = 1;
		}
	}
	pthread_rwlock_unlock(&rwlock);

	return authorized;
}

