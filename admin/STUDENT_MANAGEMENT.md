# RosForge Student Management Cheatsheet

Always run commands from the admin folder:
```bash
cd ~/ros2-teaching/admin
```

---

## Add a student

**GitHub user:**
```bash
python3 student_cli.py add Firstname Lastname --github githubusername --enrolled 2026-06-01 --payment cash --amount 890
```

**Gmail user:**
```bash
python3 student_cli.py add Firstname Lastname --gmail student@gmail.com --enrolled 2026-06-01 --payment bank --amount 890
```

Payment options: `cash`, `bank`, `transfer`, `other`, `unknown`

---

## Remove a student (keeps record, blocks access)

**GitHub user:**
```bash
python3 student_cli.py remove --github githubusername
```

**Gmail user:**
```bash
python3 student_cli.py remove --gmail student@gmail.com
```

---

## Delete a student permanently (no recovery)

**GitHub user:**
```bash
python3 student_cli.py delete --github githubusername
```

**Gmail user:**
```bash
python3 student_cli.py delete --gmail student@gmail.com
```

---

## List students

```bash
# Active students (default)
python3 student_cli.py list

# Inactive students
python3 student_cli.py list --status inactive

# All students
python3 student_cli.py list --status all
```

---

## Statistics

```bash
python3 student_cli.py stats
```

---

## Notes

- `remove` → marks student as inactive (blocked from login, record kept)
- `delete` → permanently removes the record (no recovery)
- Database is stored at `/srv/jupyterhub/data/students.db` (inside Docker volume)
- No restart needed after adding/removing students — auth checks DB live on every login
- Students log in at https://app.rosforge.com with Google or GitHub
- Admin panel: https://app.rosforge.com/hub/admin (login with github:urllib2)

